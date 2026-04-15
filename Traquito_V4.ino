// ---------------------------------------------------------------------------
// Updated executeWsprTx with "No Fix" logic
// ---------------------------------------------------------------------------
void executeWsprTx() {
    char grid4[5];

    if (havePosition) {
        char grid6[7];
        // Convert lat/lon to Maidenhead grid
        jtencode.latlon_to_grid(gpsLatDeg, gpsLonDeg, grid6);
        memcpy(grid4, grid6, 4);
        grid4[4] = '\0';
        Serial.print(F("GPS Fix Found. Transmitting Grid: "));
    } else {
        // "No Fix" substitution: We use "JJ00" or "AA00" to represent 0,0 
        // or stay with your static default. 
        // Most trackers use "AA00" to signal a coordinate error/null.
        strcpy(grid4, "AA00"); 
        Serial.print(F("No GPS Fix! Transmitting Null Grid: "));
    }
    
    Serial.println(grid4);

    uint8_t symbols[WSPR_SYMBOL_COUNT];
    jtencode.wspr_encode(CALLSIGN, grid4, WSPR_POWER_DBM, symbols);

    clockgen.enableOutputs(true);

    for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
        // Calculate frequency for the specific WSPR tone
        // Tone spacing is 1.46 Hz
        double freq = WSPR_BASE_HZ + (double)symbols[i] * (WSPR_TONE_SPACING_CENTIHZ / 100.0);
        
        // Si5351 Frequency adjustment
        // Note: For better stability, avoid resetting the PLL inside this loop
        clockgen.set_freq(freq * 100ULL, SI5351_CLK0);
        
        delay(WSPR_SYMBOL_DELAY_MS);
    }

    clockgen.enableOutputs(false);
}

// ---------------------------------------------------------------------------
// Main Loop logic (improved for reliability)
// ---------------------------------------------------------------------------
void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (c == '\n' || c == '\r') {
            nmeaBuffer[bufferIdx] = '\0';
            if (bufferIdx > 10) { // Basic check for valid string length
                parseRMC(nmeaBuffer);
            }
            bufferIdx = 0;
        } else if (bufferIdx < sizeof(nmeaBuffer) - 1) {
            nmeaBuffer[bufferIdx++] = c;
        }
    }

    // Only attempt TX if we at least have the UTC time from GPS
    if (haveTime) {
        // WSPR must start at :01 or :02 seconds into an even minute
        if (utcSecond >= 0 && utcSecond <= 2 && (utcMinute % TX_INTERVAL_MINUTES == 0)) {
            
            uint32_t currentTotalMinutes = (uint32_t)utcHour * 60 + utcMinute;
            if (currentTotalMinutes != lastTxTimeMinutes) {
                executeWsprTx();
                lastTxTimeMinutes = currentTotalMinutes;
            }
        }
    }
}