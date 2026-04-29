/*
 * Pico balloon beacon — Arduino Nano + ATGM336H-5N + Si5351A (Etherkit Library)
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <si5351.h>    // Etherkit Library
#include <JTEncode.h>

// --------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------
static const uint64_t WSPR_BASE_FREQ = 28124600ULL; // Base freq in Hz
static const int8_t WSPR_POWER_DBM = 20;
static const char CALLSIGN[] = "AK6O";

static const uint8_t TX_INTERVAL_MINUTES = 10; 
static const int PIN_GPS_RX = 1;
static const int PIN_GPS_TX = 2; 

// WSPR specific timing
static const uint16_t WSPR_SYMBOL_DELAY_MS = 683;
// Tone spacing is 1.46 Hz. In 0.01 Hz units, this is 146.
static const uint32_t TONE_STEP_CENTIHZ = 146; 

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Si5351 si5351;
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
// GPS & Helper Functions
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
    if (field[0] != 'A') { havePosition = false; return false; }

    getField(line, field, 1);
    if (strlen(field) >= 6) {
        utcHour   = (field[0] - '0') * 10 + (field[1] - '0');
        utcMinute = (field[2] - '0') * 10 + (field[3] - '0');
        utcSecond = (field[4] - '0') * 10 + (field[5] - '0');
        haveTime = true;
    }

    getField(line, field, 3);
    float lat = atof(field);
    gpsLatDeg = (int)(lat/100) + (lat - (int)(lat/100)*100)/60.0;
    getField(line, field, 4); 
    if (field[0] == 'S') gpsLatDeg = -gpsLatDeg;

    getField(line, field, 5);
    float lon = atof(field);
    gpsLonDeg = (int)(lon/100) + (lon - (int)(lon/100)*100)/60.0;
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

    Serial.print(F("TX Starting: ")); Serial.println(grid4);

    uint8_t symbols[WSPR_SYMBOL_COUNT];
    jtencode.wspr_encode(CALLSIGN, grid4, WSPR_POWER_DBM, symbols);

    // WSPR transmission loop
    for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
        // Etherkit set_freq takes frequency in 0.01 Hz units
        // Frequency = Base + (Symbol * 1.46 Hz)
        uint64_t freq = (WSPR_BASE_FREQ * 100ULL) + (uint64_t)symbols[i] * TONE_STEP_CENTIHZ;
        
        si5351.set_freq(freq, SI5351_CLK0);
        delay(WSPR_SYMBOL_DELAY_MS);
    }

    si5351.output_enable(SI5351_CLK0, 0); // Turn off clock
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
    
    // Initialize Si5351 with 8pF load (standard for Adafruit/purple modules)
    // The second 0 is for the crystal frequency (default 25MHz)
    bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
    if(!i2c_found) {
        Serial.println(F("Si5351 not found!"));
        while(1);
    }

    si5351.output_enable(SI5351_CLK0, 0);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Max power for 10m
    
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println(F("System Ready (Etherkit Mode)."));
}

void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (c == '\n') {
            if (bufferIdx > 0) {
                nmeaBuffer[bufferIdx] = '\0';
                parseRMC(nmeaBuffer);
                bufferIdx = 0;
            }       
        } else if (bufferIdx < sizeof(nmeaBuffer) - 1) {
            if (c >= 0x20 && c <= 0x7F ) nmeaBuffer[bufferIdx++] = c;
        }
    }

    if (haveTime) {
        // Checks if current second is 0 and if current minute is an interval match
        if (utcSecond == 0 && (utcMinute % TX_INTERVAL_MINUTES == 0) && (utcMinute != lastTxMinute)) {
        lastTxMinute = utcMinute; // Track that we started a TX this minute

            // Only transmit if we haven't already transmitted during THIS specific minute
            digitalWrite(LED_BUILTIN, HIGH);
            executeWsprTx();
                
            // Mark this specific minute as "done"
            digitalWrite(LED_BUILTIN, LOW);
        }
         haveTime = false;
     }
  }
