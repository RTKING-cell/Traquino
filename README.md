# Traquino

This code is a **WSPR (Weak Signal Propagation Reporter)** beacon controller for Arduino. It uses a GPS module for precise timing and location (to generate the Maidenhead locator) and an Si5351 clock generator to synthesize the RF signal.

## Features

* **GPS Time Sync:** Automatically waits for a GPS fix to ensure the WSPR transmission starts at the correct timing interval.
* **Dynamic Gridsquares:** Calculates your 4-digit Maidenhead locator (e.g., FN20) on the fly based on real-time GPS coordinates.
* **Frequency Agility:** Easily adjustable base frequency for different amateur bands.

## Hardware Requirements

* **Microcontroller:** Arduino Uno, Nano, or Pro Mini (ATmega328P).
* **Clock Generator:** Adafruit Si5351 Breakout.
* **GPS Module:** Any NMEA-compliant module (I used a ATGM336H, but NEO-6M, BN-220, and other similar chips work fine to) connected via SoftwareSerial.
* **Antenna:** Appropriate for the target band (default is 10m/28MHz).

##  Wiring

| Component | Arduino Pin | Notes |
| :--- | :--- | :--- |
| **Si5351 SDA** | A4 (SDA) | I2C Data |
| **Si5351 SCL** | A5 (SCL) | I2C Clock |
| **GPS TX** | Pin 4 | Data from GPS to Arduino |
| **GPS RX** | Pin 5 | (Unused by code but connected) |

##  Configuration

Before uploading, modify the "User Configuration" section in the code:

```cpp
static const uint64_t WSPR_BASE_FREQ = 28124600ULL; // Base freq in Hz
static const int8_t WSPR_POWER_DBM = 20;
static const char CALLSIGN[] = "NOCALL";

static const uint8_t TX_INTERVAL_MINUTES = 10; 
static const int PIN_GPS_RX = 1;
static const int PIN_GPS_TX = 2; 
```

## Required Libraries

You must install these via the Arduino Library Manager:
1. `Si5351` (by Etherkit)
2. `JTEncode` (by Etherkit)
3. `SoftwareSerial` (built-in)
4. `Wire` (built-in)

##  How it Works

1.  **Boot:** The system checks the Si5351 and waits for a valid GPS fix.
2.  **Date Check:** It reads the EEPROM to see the last time it transmitted.
3.  **Scheduling:** * If current UTC time is between `START_HOUR` and `END_HOUR`...
    * ...and it hasn't transmitted yet today...
    * ...it generates the WSPR symbols and begins the ~110-second transmission.
4.  **Idle:** After transmitting (or if outside the window), it enters a low-activity loop, checking the GPS every minute to keep time synchronized.

## Disclaimer
*This software is for licensed Amateur Radio operators only. Ensure your hardware includes proper low-pass filtering before connecting to an antenna to prevent interference.
Also when inputing TX_INTERVAL_MINUTES, make sure it is an even number, or else the logic breaks.*
