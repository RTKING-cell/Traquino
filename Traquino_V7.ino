/*
 * Pico balloon beacon — Arduino Nano Every (ATmega4809)
 * Powered directly via 3.3V Pin (No Voltage Divider Required)
 * ATGM336H-5N + Si5351A (Etherkit Library)
 * Integrated with ArduinoLowPower.h & Pre-TX Brownout Guard
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <si5351.h>          // Etherkit Library
#include <JTEncode.h>
#include <ArduinoLowPower.h> // Official Arduino Low Power Library

// --------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------
static const uint64_t WSPR_BASE_FREQ = 28124600ULL; // Base freq in Hz
static const int8_t WSPR_POWER_DBM = 20;
static const char CALLSIGN[] = "AK6O";

static const uint8_t TX_INTERVAL_MINUTES = 10; 
static const int PIN_GPS_RX = 1;
static const int PIN_GPS_TX = 2; 

// Brownout & Power Management Thresholds
static const float SAFE_VOLTAGE_THRESHOLD = 3.00; // Min supply voltage (V) to attempt TX (Adjust for 3.3V rail)
static const float HYSTERESIS_VOLTAGE      = 0.20; // Recovery buffer voltage (V)

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



float readSupplyVoltage() {
  // Configure ATmega4809 internal ADC: measure VDD directly against 2.5V VREF
  VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLA = ADC_ENABLE_bm;
  
  // Start ADC conversion
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)); // Wait for conversion completion
  
  uint16_t adcResult = ADC0.RES;
  
  // Disable ADC module after read to conserve energy
  ADC0.CTRLA &= ~ADC_ENABLE_bm;

  if (adcResult == 0) return 0.0;

  // Calculate actual VDD supply voltage in Volts
  float vdd = (2.5 * 1024.0) / adcResult;
  return vdd;
}

// ---------------------------------------------------------------------------
// Low Power & Brownout Guard using ArduinoLowPower
// ---------------------------------------------------------------------------
bool checkPowerState() {
  float vcc = readSupplyVoltage();

  if (vcc < SAFE_VOLTAGE_THRESHOLD) {
    Serial.print(F("Low Vcc detected: "));
    Serial.print(vcc);
    Serial.println(F("V. Shutting down RF clock and sleeping..."));
    Serial.flush();

    // Kill Si5351 output clock immediately to stop RF current drain
    si5351.output_enable(SI5351_CLK0, 0);

    // Deep sleep using ArduinoLowPower until solar/battery rail recovers
    while (vcc < (SAFE_VOLTAGE_THRESHOLD + HYSTERESIS_VOLTAGE)) {
      LowPower.deepSleep(8000); // Deep sleep 8 seconds
      vcc = readSupplyVoltage(); // Sample VDD internally upon waking
    }

    Serial.println(F("Rail voltage recovered. Resuming main loop..."));
    return false; // Skip the scheduled TX cycle
  }

  return true; // Safe operating voltage
}

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
    // Pre-TX Check: Verify Vcc before powering RF transmit stage
    if (!checkPowerState()) {
      Serial.println(F("TX Aborted: Low Vcc condition."));
      return;
    }

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

    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
    si5351.output_enable(SI5351_CLK0, 1);

    // WSPR transmission loop
    for (uint8_t i = 0; i < WSPR_SYMBOL_COUNT; i++) {
        uint64_t freq = (WSPR_BASE_FREQ * 100ULL) + (uint64_t)symbols[i] * TONE_STEP_CENTIHZ;
        
        si5351.set_freq(freq, SI5351_CLK0);
        delay(WSPR_SYMBOL_DELAY_MS);
    }

    si5351.output_enable(SI5351_CLK0, 0); // Turn off transmitter clock
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
    
    // Initialize Si5351 with 8pF load
    bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
    if(!i2c_found) {
        Serial.println(F("Si5351 not found!"));
        while(1);
    }

    si5351.output_enable(SI5351_CLK0, 0);
    
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println(F("Nano Every Ready (Direct 3.3V Mode + Brownout Active)."));
}

void loop() {
    // 1. Guard check on main loop iteration
    checkPowerState();

    // 2. Process incoming NMEA sentences from ATGM336H GPS
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

    // 3. Trigger WSPR transmission on exact scheduled interval
    if (haveTime) {
        if (utcSecond == 0 && (utcMinute % TX_INTERVAL_MINUTES == 0) && (utcMinute != lastTxMinute)) {
            lastTxMinute = utcMinute;

            digitalWrite(LED_BUILTIN, HIGH);
            executeWsprTx();
            digitalWrite(LED_BUILTIN, LOW);
        }
        haveTime = false;
    }
}