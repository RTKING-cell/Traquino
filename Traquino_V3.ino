#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_SI5351.h>
#include <JTEncode.h>

// ---------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------
static const double WSPR_BASE_HZ = 28124600.0;
static const int8_t WSPR_POWER_DBM = 20;
static const char CALLSIGN[] = "NOCALL";
static const char GRID_STATIC[] = "AA00";

// Solar Strategy: Transmit every X minutes (must be an even number for WSPR)
static const uint8_t TX_INTERVAL_MINUTES = 10; 

static const int PIN_GPS_RX = 4;
static const int PIN_GPS_TX = 5; 
static const long GPS_BAUD = 9600;

static const uint16_t WSPR_SYMBOL_DELAY_MS = 683;
static const uint8_t WSPR_TONE_SPACING_CENTIHZ = 146;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Adafruit_SI5351 clockgen;
SoftwareSerial gpsSerial(PIN_GPS_RX, PIN_GPS_TX);
JTEncode jtencode;

uint8_t utcHour, utcMinute, utcSecond;
float gpsLatDeg, gpsLonDeg;
bool haveTime = false;
bool havePosition = false;
uint32_t lastTxTimeMinutes = 0; // Tracks last TX to prevent rapid firing

char nmeaBuffer[100];
byte bufferIdx = 0;

// ---------------------------------------------------------------------------
// GPS & TX Functions (Simplified for Solar)
// ---------------------------------------------------------------------------

void getField(char* buffer, char* field, int fieldNum) {
    int count = 0, i = 0, j = 0;
    while (count < fieldNum && buffer[i] != '\0') {
        if (buffer[i] == ',') count++;
        i++;
    }
    while (buffer[i] != ',' && buffer[i] != '\0' && buffer[i] != '*') {
        field[j++] = buffer[i++];
    }
    field[j] = '\0';
}

bool parseRMC(char* line) {
    if (strncmp(line, "$GNRMC", 6) != 0 && strncmp(line, "$GPRMC", 6) != 0) return false;
    char field[16];
    
    getField(line, field, 2);
    if (field[0] != 'A') return false; 

    getField(line, field, 1);
    if (strlen(field) >= 6) {
        utcHour   = (field[0] - '0') * 10 + (field[1] - '0');
        utcMinute = (field[2] - '0') * 10 + (field[3] - '0');
        utcSecond = (field[4] - '0') * 10 + (field[5] - '0');
    }

    getField(line, field, 3);
    float lat = atof(field);
    getField(line, field, 4);
    int deg = (int)(lat / 100);
    gpsLatDeg = deg + (lat - deg * 100) / 60.0;
    getField(line, field, 4); 
    if (field[0] == 'S') gpsLatDeg = -gpsLatDeg;

    getField(line, field, 5);
    float lon = atof(field);
    getField(line, field, 6);
    deg = (int)(lon / 100);
    gpsLonDeg = deg + (lon - deg * 100) / 60.0;
    if (field[0] == 'W') gpsLonDeg = -gpsLonDeg;

    haveTime = true;
    havePosition = true;
    return true;
}

//Tx Helper Functions
void executeWsprTx() {
    char grid4[5];
    if (havePosition) {
        char grid6[7];
        jtencode.latlon_to_grid(gpsLatDeg, gpsLonDeg, grid6);
        memcpy(grid4, grid6, 4);
        grid4[4] = '\0';
    } else {
        strcpy(grid4, GRID_STATIC);
    }

    Serial.print(F("Solar TX Starting: "));
    Serial.println(grid4);

    uint8_t symbols[WSPR_SYMBOL_COUNT];
    jtencode.wspr_encode(CALLSIGN, grid4, WSPR_POWER_DBM, symbols);

    // Using CLK0 (output 0)
    clockgen.enableOutputs(true);

    for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
        // Calculate the target frequency for this symbol
        double freq = WSPR_BASE_HZ + (double)symbols[i] * (WSPR_TONE_SPACING_CENTIHZ / 100.0);
        
        // In the Adafruit library, we set the frequency by configuring the Multisynth.
        // For 28MHz (10m band), a simple setup works:
        // We set the PLL to a fixed value (e.g., 900MHz) and divide down.
        uint32_t pllFreq = 900000000UL;
        clockgen.setupPLL(SI5351_PLL_A, 36, 0, 1); // 25MHz * 36 = 900MHz
        
        // Set the divider for the specific frequency
        // setupMultisynth(output_channel, pll, divider, numerator, denominator)
        uint32_t divider = pllFreq / (uint32_t)freq;
        clockgen.setupMultisynth(0, SI5351_PLL_A, divider, 0, 1);
        
        delay(WSPR_SYMBOL_DELAY_MS);
    }

    clockgen.enableOutputs(false);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(GPS_BAUD);
    Wire.begin();

    if (clockgen.begin() != ERROR_NONE) {
        while (1); 
    }
    clockgen.enableOutputs(false);
    Serial.println(F("Waiting for GPS..."));
}

void loop() {
    // Process GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (c == '\n' || c == '\r') {
            nmeaBuffer[bufferIdx] = '\0';
            parseRMC(nmeaBuffer);
            bufferIdx = 0;
        } else if (bufferIdx < sizeof(nmeaBuffer) - 1) {
            nmeaBuffer[bufferIdx++] = c;
        }
    }

    if (haveTime) {
        // WSPR Logic for Solar:
        // 1. Must be the start of an even minute (:00 seconds)
        // 2. Must meet our interval (e.g., every 10 minutes)
        if (utcSecond == 0 && (utcMinute % TX_INTERVAL_MINUTES == 0)) {
            
            // Prevent re-triggering within the same minute
            uint32_t currentTotalMinutes = (uint32_t)utcHour * 60 + utcMinute;
            if (currentTotalMinutes != lastTxTimeMinutes) {
                executeWsprTx();
                lastTxTimeMinutes = currentTotalMinutes;
            }
        }
    }
}