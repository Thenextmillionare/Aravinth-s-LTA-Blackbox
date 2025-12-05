// SIM7600E AV Telematics (1 Hz) – SAFE VERSION
// - Lat/Lon, speed, acceleration
// - Smoothed speed + robust STOPPED/MOVING state
// - Harsh braking & rapid acceleration detection
// - Speeding detection (configurable)
// - Distance, moving/stopped time, event counters
// - Type 'x' or 'X' in Serial Monitor to halt and print summary
// - Extra sanity checks to avoid crazy speeds/accels when stationary

#include <math.h>

// --- Configurable parameters ---

// State thresholds (km/h)
const double STOP_THRESHOLD_KMH           = 1.5;
const double START_MOVE_THRESHOLD_KMH     = 4.0;

// Samples needed to change state
const int SAMPLES_TO_CONFIRM_STOP         = 5;   // ~5 s
const int SAMPLES_TO_CONFIRM_MOVE         = 3;   // ~3 s

// Speed smoothing (EMA)
const double SPEED_SMOOTH_ALPHA           = 0.3;

// Harsh event thresholds
const double HARSH_BRAKE_THRESH_MPS2      = -3.0;  // strong braking
const double RAPID_ACCEL_THRESH_MPS2      =  3.0;  // strong accel
const double MIN_SPEED_FOR_EVENT_KMH      = 10.0;  // ignore when almost stopped

// Speeding detection
const double SPEED_LIMIT_KMH              = 50.0;  // adjust to test-route limit
const double SPEEDING_TOLERANCE_KMH       = 2.0;   // small margin

// Absolute sanity limit for speeds (anything above is treated as invalid)
const double MAX_REASONABLE_SPEED_KMH     = 200.0;

// --- State variables ---

// GPS fix history
bool   hasPrevFix    = false;
double prevLat       = 0.0;
double prevLon       = 0.0;
unsigned long prevTimeMs = 0;

// Speed history
double filteredSpeedKmh      = 0.0;
double prevFilteredSpeedKmh  = 0.0;
bool   hasPrevSpeed          = false;

// Vehicle state machine
enum VehicleState {
  VS_UNKNOWN,
  VS_STOPPED,
  VS_MOVING
};

VehicleState currentState = VS_UNKNOWN;
int lowSpeedCount  = 0;
int highSpeedCount = 0;

// Trip statistics
double totalDistanceM   = 0.0;
unsigned long movingTimeMs  = 0;
unsigned long stoppedTimeMs = 0;

int harshBrakeCount     = 0;
int rapidAccelCount     = 0;
int speedingEventCount  = 0;
bool isSpeedingNow      = false;

// Last known timestamp from GPS
String lastTimestamp = "UNKNOWN";


// --- Helper functions ---

// Read one line from SIM7600E (terminated by '\n' or timeout)
String readLineFromSIM() {
  String line = "";
  unsigned long start = millis();
  while (millis() - start < 2000) { // 2-sec timeout
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') return line;
      if (c != '\r') line += c;
    }
  }
  return line;
}

// Safe ddmm.mmmm or dddmm.mmmm -> decimal degrees
double convertToDecimal(String val, char hemi) {
  int dotIndex = val.indexOf('.');
  if (dotIndex < 0) return NAN;         // no decimal point
  if (dotIndex < 3) return NAN;         // too short to have deg+min

  int degLen = dotIndex - 2;            // ddmm.mmmm or dddmm.mmmm
  if (degLen <= 0) return NAN;

  double degrees = val.substring(0, degLen).toDouble();
  double minutes = val.substring(degLen).toDouble();

  double decimal = degrees + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') decimal = -decimal;

  // Extra sanity: latitude [-90,90], longitude [-180,180]
  if (abs(decimal) > 180.0) return NAN;
  return decimal;
}

double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius (m)

  double radLat1 = lat1 * PI / 180.0;
  double radLat2 = lat2 * PI / 180.0;
  double dLat    = (lat2 - lat1) * PI / 180.0;
  double dLon    = (lon2 - lon1) * PI / 180.0;

  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(radLat1) * cos(radLat2) *
             sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

const char* stateToString(VehicleState st) {
  switch (st) {
    case VS_STOPPED: return "STOPPED";
    case VS_MOVING:  return "MOVING";
    default:         return "UNKNOWN";
  }
}

// Format GPS date/time into YYYY-MM-DD HH:MM:SS (basic checks)
String formatTimestamp(String dateStr, String timeStr) {
  // Expect DDMMYY and HHMMSS.s or HHMMSS
  if (dateStr.length() != 6 || timeStr.length() < 6) return "UNKNOWN";
  if (!isDigit(timeStr[0]) || !isDigit(timeStr[1])) return "UNKNOWN";

  String dd = dateStr.substring(0, 2);
  String MM = dateStr.substring(2, 4);
  String yy = dateStr.substring(4, 6); // 00–99

  String hh = timeStr.substring(0, 2);
  String mm = timeStr.substring(2, 4);
  String ss = timeStr.substring(4, 6);

  int year = 2000 + yy.toInt();
  String ts = String(year) + "-" + MM + "-" + dd +
              " " + hh + ":" + mm + ":" + ss;
  return ts;
}

// Print trip summary on halt
void printSummary() {
  Serial.println();
  Serial.println("========== TEST SUMMARY ==========");
  Serial.print("Total distance: ");
  Serial.print(totalDistanceM / 1000.0, 3);
  Serial.println(" km");

  Serial.print("Moving time: ");
  Serial.print(movingTimeMs / 1000.0);
  Serial.println(" s");

  Serial.print("Stopped time: ");
  Serial.print(stoppedTimeMs / 1000.0);
  Serial.println(" s");

  Serial.print("Harsh braking events: ");
  Serial.println(harshBrakeCount);

  Serial.print("Rapid acceleration events: ");
  Serial.println(rapidAccelCount);

  Serial.print("Speeding events: ");
  Serial.println(speedingEventCount);

  Serial.print("Last GPS timestamp: ");
  Serial.println(lastTimestamp);
  Serial.println("==================================");
}


// --- Arduino setup/loop ---

void setup() {
  Serial.begin(115200);   // USB to PC
  Serial1.begin(115200);  // SIM7600E UART

  delay(2000);
  Serial.println("SIM7600E AV Telematics (1 Hz) – Safe Version");
  Serial.println("Type 'x' or 'X' in Serial Monitor to halt and print summary.");

  // Turn GPS on (ignore ERROR if already on)
  Serial1.println("AT+CGPS=1");
  delay(1500);
}

void loop() {
  // USER STOP COMMAND
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') {
      Serial.println(">>> SYSTEM HALTED BY USER <<<");
      printSummary();
      while (true) {} // freeze until reset
    }
  }

  // Request GPS info
  Serial1.println("AT+CGPSINFO");
  delay(500);

  String line = readLineFromSIM();

  if (line.startsWith("+CGPSINFO:")) {
    String data = line.substring(String("+CGPSINFO:").length());
    data.trim();

    // Split into CSV fields
    String fields[10];
    int fieldIndex = 0;
    for (int i = 0; i < 10; i++) fields[i] = "";

    for (int i = 0; i < data.length(); i++) {
      char c = data[i];
      if (c == ',') {
        fieldIndex++;
        if (fieldIndex >= 10) break;
      } else {
        fields[fieldIndex] += c;
      }
    }

    String latStr  = fields[0];
    char   latHem  = fields[1].length() ? fields[1][0] : 'N';
    String lonStr  = fields[2];
    char   lonHem  = fields[3].length() ? fields[3][0] : 'E';
    String dateStr = fields[4];   // DDMMYY
    String timeStr = fields[5];   // HHMMSS.s (usually)

    // No fix yet → lat/lon empty
    if (latStr.length() == 0 || lonStr.length() == 0) {
      Serial.println("No GPS fix yet... (empty lat/lon)");
      delay(1000);
      return;
    }

    double lat = convertToDecimal(latStr, latHem);
    double lon = convertToDecimal(lonStr, lonHem);

    if (isnan(lat) || isnan(lon)) {
      Serial.println("Invalid lat/lon from GPS, skipping this sample.");
      delay(1000);
      return;
    }

    // Update human-readable timestamp (may still be UNKNOWN if bad)
    lastTimestamp = formatTimestamp(dateStr, timeStr);

    unsigned long nowMs = millis();
    double instantSpeedKmh = 0.0;
    double dtSec = 0.0;
    double distM = 0.0;
    bool speedValid = false;   // will become true if calcs look sane

    // Compute distance & instant speed if we have a previous fix
    if (hasPrevFix) {
      dtSec = (nowMs - prevTimeMs) / 1000.0;
      if (dtSec > 0.3) {
        distM = distanceMeters(prevLat, prevLon, lat, lon);

        // Very small jumps when stationary are just jitter; allow them
        double speedMps = distM / dtSec;
        instantSpeedKmh = speedMps * 3.6;

        // Basic sanity clamp on instant speed
        if (instantSpeedKmh >= 0.0 && instantSpeedKmh <= MAX_REASONABLE_SPEED_KMH) {
          speedValid = true;
        } else {
          Serial.print("Discarding unrealistic speed sample: ");
          Serial.print(instantSpeedKmh);
          Serial.println(" km/h");
          instantSpeedKmh = 0.0;
          speedValid = false;
        }
      }
    }

    // Update previous GPS fix
    prevLat    = lat;
    prevLon    = lon;
    prevTimeMs = nowMs;
    hasPrevFix = true;

    // Only accumulate distance if this sample looks sane
    if (speedValid) {
      totalDistanceM += distM;
    }

    // Smooth speed (EMA)
    if (speedValid) {
      if (!hasPrevSpeed) {
        filteredSpeedKmh = instantSpeedKmh;
        hasPrevSpeed = true;
      } else {
        filteredSpeedKmh =
          (1.0 - SPEED_SMOOTH_ALPHA) * filteredSpeedKmh +
           SPEED_SMOOTH_ALPHA        * instantSpeedKmh;
      }
    }

    // Extra safety: clamp filtered speed to a reasonable range
    if (filteredSpeedKmh < 0.0 || filteredSpeedKmh > MAX_REASONABLE_SPEED_KMH) {
      filteredSpeedKmh = 0.0;
    }

    // Compute acceleration (based on filtered speed)
    double accelMps2 = 0.0;
    if (hasPrevSpeed && dtSec > 0.3 && speedValid) {
      double dvKmh = filteredSpeedKmh - prevFilteredSpeedKmh;
      double dvMps = dvKmh / 3.6;
      accelMps2    = dvMps / dtSec;

      if (filteredSpeedKmh > MIN_SPEED_FOR_EVENT_KMH) {
        if (accelMps2 < HARSH_BRAKE_THRESH_MPS2) {
          harshBrakeCount++;
          Serial.print("ALERT: Harsh braking! a = ");
          Serial.print(accelMps2, 2);
          Serial.println(" m/s^2");
        } else if (accelMps2 > RAPID_ACCEL_THRESH_MPS2) {
          rapidAccelCount++;
          Serial.print("ALERT: Rapid acceleration! a = ");
          Serial.print(accelMps2, 2);
          Serial.println(" m/s^2");
        }
      }
    }
    prevFilteredSpeedKmh = filteredSpeedKmh;

    // Update STOPPED/MOVING state using filtered speed
    double stateSpeed = filteredSpeedKmh;

    if (stateSpeed < STOP_THRESHOLD_KMH) {
      lowSpeedCount++;
      highSpeedCount = 0;
      if (lowSpeedCount >= SAMPLES_TO_CONFIRM_STOP &&
          currentState != VS_STOPPED) {
        currentState = VS_STOPPED;
      }
    } else if (stateSpeed > START_MOVE_THRESHOLD_KMH) {
      highSpeedCount++;
      lowSpeedCount = 0;
      if (highSpeedCount >= SAMPLES_TO_CONFIRM_MOVE &&
          currentState != VS_MOVING) {
        currentState = VS_MOVING;
      }
    } else {
      lowSpeedCount  = 0;
      highSpeedCount = 0;
    }

    // Accumulate moving / stopped time
    if (dtSec > 0.0) {
      if (currentState == VS_MOVING) {
        movingTimeMs += (unsigned long)(dtSec * 1000.0);
      } else if (currentState == VS_STOPPED) {
        stoppedTimeMs += (unsigned long)(dtSec * 1000.0);
      }
    }

    //  Speeding detection with hysteresis
    double speedLimitHigh = SPEED_LIMIT_KMH + SPEEDING_TOLERANCE_KMH;
    if (!isSpeedingNow &&
        stateSpeed > speedLimitHigh &&
        currentState == VS_MOVING) {
      isSpeedingNow = true;
      speedingEventCount++;
      Serial.print("ALERT: Speeding! v = ");
      Serial.print(stateSpeed, 2);
      Serial.print(" km/h (limit ");
      Serial.print(SPEED_LIMIT_KMH);
      Serial.println(")");
    } else if (isSpeedingNow &&
               stateSpeed < SPEED_LIMIT_KMH) {
      // dropped back below limit -> clear flag
      isSpeedingNow = false;
    }

    // --- Main telemetry line ---
    Serial.print("T: ");
    Serial.print(lastTimestamp);
    Serial.print("  Lat: ");
    Serial.print(lat, 6);
    Serial.print("  Lon: ");
    Serial.print(lon, 6);
    Serial.print("  Inst: ");
    Serial.print(instantSpeedKmh, 2);
    Serial.print(" km/h  Filt: ");
    Serial.print(filteredSpeedKmh, 2);
    Serial.print(" km/h  a: ");
    Serial.print(accelMps2, 2);
    Serial.print(" m/s^2  State: ");
    Serial.println(stateToString(currentState));
  }

  delay(1000);  // 1 Hz
}
