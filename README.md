# Traquino

This code is a **WSPR (Weak Signal Propagation Reporter)** beacon controller for Arduino. It uses a GPS module for precise timing and location (to generate the Maidenhead locator) and an Si5351 clock generator to synthesize the RF signal.

The logic includes a specific "Morning Window" transmission feature to save power or bandwidth, only transmitting once per day during a specific UTC time block.


## Features

* **GPS Time Sync:** Automatically waits for a GPS fix to ensure the WSPR transmission starts at the correct timing interval.
* **Dynamic Gridsquares:** Calculates your 4-digit Maidenhead locator (e.g., FN20) on the fly based on real-time GPS coordinates.
* **Scheduled Transmissions:** Configurable UTC "Morning Window." The beacon will only transmit once per day within this window.
* **Persistence:** Uses EEPROM to store the last transmission date, ensuring the beacon doesn't double-transmit if the power is cycled.
* **Frequency Agility:** Easily adjustable base frequency for different amateur bands.

## Hardware Requirements

* **Microcontroller:** Arduino Uno, Nano, or Pro Mini (ATmega328P).
* **Clock Generator:** Adafruit Si5351 Breakout.
* **GPS Module:** Any NMEA-compliant module (e.g., NEO-6M, BN-220) connected via SoftwareSerial.
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
static const double WSPR_BASE_HZ = 28124600.0; // Target frequency
static const char CALLSIGN[] = "NOCALL";       // Your Ham Radio Callsign
static const int8_t WSPR_POWER_DBM = 20;       // Reported Power (dBm)
static const uint8_t MORNING_START_UTC_HOUR = 6;
static const uint8_t MORNING_END_UTC_HOUR = 12;
```

## Required Libraries

You must install these via the Arduino Library Manager:
1. `Adafruit_Si5351`
2. `JTEncode` (by Etherkit)
3. `SoftwareSerial` (built-in)
4. `Wire` and `EEPROM` (built-in)

##  How it Works

1.  **Boot:** The system checks the Si5351 and waits for a valid GPS fix.
2.  **Date Check:** It reads the EEPROM to see the last time it transmitted.
3.  **Scheduling:** * If current UTC time is between `START_HOUR` and `END_HOUR`...
    * ...and it hasn't transmitted yet today...
    * ...it generates the WSPR symbols and begins the ~110-second transmission.
4.  **Idle:** After transmitting (or if outside the window), it enters a low-activity loop, checking the GPS every minute to keep time synchronized.

## Disclaimer
*This software is for licensed Amateur Radio operators only. Ensure your hardware includes proper low-pass filtering before connecting to an antenna to prevent interference.*
