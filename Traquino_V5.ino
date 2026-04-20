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
static const char CALLSIGN[] = "AK6O";
static const char GRID_STATIC[] = "CM97";

// Solar Strategy
static const uint8_t TX_INTERVAL_MINUTES = 10; // Start a TX every X minutes
static const uint8_t TX_DURATION_MINUTES = 2;  // WSPR standard is 2 minutes

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
int lastTxMinute = -1; 

char nmeaBuffer[100];
byte bufferIdx = 0;

// ---------------------------------------------------------------------------
// GPS & TX Functions
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
    char field[16];
    if (strncmp(line, "$GNRMC", 6) != 0 && strncmp(line, "$GPRMC", 6) != 0) return false;
    
    getField(line, field, 2);
    if (field[0] != 'A') {
        havePosition = false; 
        return false; 
    }

    getField(line, field, 1);
    if (strlen(field) >= 6) {
        utcHour   = (field[0] - '0') * 10 + (field[1] - '0');
        utcMinute = (field[2] - '0') * 10 + (field[3] - '0');
        utcSecond = (field[4] - '0') * 10 + (field[5] - '0');
        haveTime = true;
    }

    getField(line, field, 3);
    float lat = atof(field);
    int deg = (int)(lat / 100);
    gpsLatDeg = deg + (lat - deg * 100) / 60.0;
    getField(line, field, 4); 
    if (field[0] == 'S') gpsLatDeg = -gpsLatDeg;

    getField(line, field, 5);
    float lon = atof(field);
    deg = (int)(lon / 100);
    gpsLonDeg = deg + (lon - deg * 100) / 60.0;
    getField(line, field, 6);
    if (field[0] == 'W') gpsLonDeg = -gpsLonDeg;

    havePosition = true;
    return true;
}

void executeWsprTx() {
    char grid4[5];
    if (havePosition) {
        char grid6[7];
        jtencode.latlon_to_grid(gpsLatDeg, gpsLonDeg, grid6);
        memcpy(grid4, grid6, 4);
        grid4[4] = '\0';
    } else {
        strcpy(grid4, "AA00"); 
    }

    Serial.print(F("TX Starting: "));
    Serial.println(grid4);

    uint8_t symbols[WSPR_SYMBOL_COUNT];
    jtencode.wspr_encode(CALLSIGN, grid4, WSPR_POWER_DBM, symbols);

    clockgen.setupPLL(SI5351_PLL_A, 36, 0, 1); 
    clockgen.enableOutputs(true);

    for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
        double freq = WSPR_BASE_HZ + (double)symbols[i] * (WSPR_TONE_SPACING_CENTIHZ / 100.0);
        uint32_t divider = 900000000UL / (uint32_t)freq;
        uint32_t residual = 900000000UL % (uint32_t)freq;
        uint32_t lNum = (uint32_t)(((uint64_t)residual * 1000000ULL) / (uint32_t)freq);
        
        clockgen.setupMultisynth(0, SI5351_PLL_A, divider, lNum, 1000000);
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
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println(F("System Ready."));
}

void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (c == '\n' || c == '\r') {
            if (bufferIdx > 0) {
                nmeaBuffer[bufferIdx] = '\0';
                parseRMC(nmeaBuffer);
                bufferIdx = 0;
            }
        } else if (bufferIdx < sizeof(nmeaBuffer) - 1) {
            nmeaBuffer[bufferIdx++] = c;
        }
    }

    if (haveTime) {
        // Condition 1: Must be the 0th second of a minute
        // Condition 2: Minute must be a multiple of the Interval
        // Condition 3: Must not have transmitted in this slot yet
        if (utcSecond == 0 && (utcMinute % TX_INTERVAL_MINUTES == 0)) {
            
            // Check to see if we have already handled this minute slot
            if (utcMinute != lastTxMinute) {
                digitalWrite(LED_BUILTIN, HIGH);
                executeWsprTx();
                
                // Record the minute we started TX so we don't repeat
                lastTxMinute = utcMinute; 
                digitalWrite(LED_BUILTIN, LOW);
            }
        }
    }
}