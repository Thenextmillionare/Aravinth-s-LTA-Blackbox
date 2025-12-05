# Aravinth’s LTA Blackbox (SIM7600E + Arduino Mega)

A self contained GNSS based telematics blackbox designed for autonomous vehicle (AV) testing.
Built using the SIM7600E GNSS module and Arduino Mega, the system provides reliable 1 Hz GPS analytics, speed estimation, acceleration detection, motion classification, and end-of-test reporting: fully independent of vehicle CAN or onboard sensors.

# Overview

This project implements a complete GPS analytics pipeline on an Arduino Mega, transforming raw GNSS readings into stable, high quality telemetric data useful for AV validation and road test logging.
The system runs entirely on external GNSS and 5V power, making it portable and vehicle agnostic.

# Core Features

1. Robust GNSS Parsing

Reads GNSS output using AT+CGPSINFO from the SIM7600E.

Converts coordinates from DDM (Degrees + Decimal Minutes) to decimal degrees.

Rejects malformed, empty, or impossible coordinates.

Ensures downstream computations receive only clean data.


2. Accurate Speed Estimation

Computes speed using Haversine distance / Δt, not SIM7600E’s internal speed.

Prevents reliance on vendor-specific GNSS speed fields.

Caps unrealistic values (e.g., >200 km/h) to prevent noise artifacts.


3. Exponential Smoothing (EMA)

Provides stable “Filtered Speed” by reducing GNSS jitter:

Eliminates false acceleration spikes.

Smooths raw instantaneous speed into a reliable signal for analytics.

Tunable smoothing constant (SPEED_SMOOTH_ALPHA).


4. Acceleration & Event Detection

Acceleration is computed from filtered speed (Δv / Δt), enabling:

Harsh braking detection (< –3.0 m/s²)

Rapid acceleration detection (> +3.0 m/s²)

Speeding detection with configurable limit + tolerance

Event hysteresis prevents duplicate detection


5. Motion Classification

A state machine classifies:

STOPPED: confirmed after multiple low-speed readings

MOVING: confirmed after sustained high-speed readings
Uses sample windows (e.g., 5s) to prevent flicker due to GPS noise.


6. Trip Statistics

Automatically tracks:

Total distance traveled

Total moving time

Total stopped time

Count of braking, acceleration, and speeding events


7. End-of-Test Summary

Typing x or X in the Serial Monitor:

Safely halts the blackbox

Prints a complete summary report

Freezes execution until a manual reset
This ensures clean and controlled logging of each test run.


# Hardware Requirements

Arduino Mega 2560

SIM7600E GNSS/LTE module

GPS patch antenna (U.FL ceramic recommended)

Stable 5V power source (powerbank or vehicle feed)

# Wiring Guide
SIM7600E TXD → Mega RX1 (Pin 19)
SIM7600E RXD → Mega TX1 (Pin 18)
SIM7600E GND → Mega GND
SIM7600E VCC → Mega 5V
GPS Antenna → GNSS port

# How It Works

SIM7600E activated with AT+CGPS=1

GNSS fix polled at 1 Hz

Raw lat/lon parsed and validated

Coordinates converted from DDM → decimal degrees

Distance computed via Haversine

Speed = distance / time

Speed smoothed with EMA

Acceleration derived from Δv

State machine + event detection

Telemetry printed live & summary printed on halt

📄 Example Telemetry Output
T: 2025-12-02 14:12:09  
Lat: 1.345678  Lon: 103.789012  
Inst: 0.08 km/h  Filt: 0.02 km/h  
a: 0.00 m/s^2  State: STOPPED


Example Event:

ALERT: Harsh braking! a = -3.24 m/s^2


Summary Report:

========== TEST SUMMARY ==========
Total distance: 1.342 km
Moving time: 301 s
Stopped time: 120 s
Harsh braking events: 1
Rapid acceleration events: 2
Speeding events: 0
==================================

# Intended Use Cases

AV (autonomous vehicle) development and validation

Benchmarking braking/accel behaviour

Blackbox-style logging for road trials

Portable instrumentation without access to CAN/vehicle sensors

Telematics research and prototyping


# Limitations & Considerations

GNSS performance depends on sky visibility

No IMU: acceleration is derived from GPS, not inertial sensing

1 Hz update rate (expandable depending on SIM7600E firmware capabilities)

Indoor performance limited (GNSS only, no hybrid positioning)


# Future Enhancements (Optional)

5 Hz GNSS polling (if supported by module firmware)

SD-card logging

LTE upload of events

IMU fusion for higher-fidelity acceleration

Onboard real-time clock backup

Zonal speed detection
