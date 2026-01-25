#include <Adafruit_TinyUSB.h>
#include <LSM6DS3.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <bluefruit.h>

// --- GPS Configuration ---
// On XIAO nRF52840:
// Serial1 TX is Pin D6 -> Connect to GPS RX
// Serial1 RX is Pin D7 -> Connect to GPS TX
#define GPS_BAUD 115200
#define FACTORY_GPS_BAUD 9600
#define MAX_NAVIGATION_RATE 25

SFE_UBLOX_GNSS myGNSS;

// --- IMU Configuration (Onboard LSM6DS3) ---
// XIAO Sense IMU is on I2C address 0x6A
LSM6DS3 IMU(I2C_MODE, 0x6A);
#define int1Pin PIN_LSM6DS3TR_C_INT1

// sleep stuff
#define GPS_EN_PIN D1
const unsigned long SLEEP_TIMEOUT = 300000; // 5 minutes
#define PIN_VBAT (32)                       // D32 battery voltage
#define PIN_VBAT_ENABLE (14)                // D14 LOW:read anable
#define PIN_HICHG (22) // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG (23)   // D23 charge indicatore LOW:charge HIGH:no charge

// --- GNSS Constellations ---
#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
#define ENABLE_GNSS_GLONASS
#define ENABLE_GNSS_BEIDOU
// #define ENABLE_GNSS_SBAS
// #define ENABLE_GNSS_QZSS

const char *rawDeviceName = "RaceBox Mini 0123456789";
const String deviceName = rawDeviceName;

// LED Configuration (Red LED on XIAO)
const int OnboardledPin = LED_BLUE;

const unsigned long AccelSampleInterval = 10; // 10ms = 100Hz
bool lastChargingState = false;
// --- Smoothing Configuration ---
float accelAlpha = 0.8;
float gyroAlpha = 0.8;
// Storage for the filtered values
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// --- BLE UUIDs ---
// Bluefruit handles UUIDs as objects
const uint8_t RACEBOX_SERVICE_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                        0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                        0x01, 0x00, 0x40, 0x6E};

const uint8_t RACEBOX_CHAR_TX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                        0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                        0x03, 0x00, 0x40, 0x6E};

const uint8_t RACEBOX_CHAR_RX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                        0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                        0x02, 0x00, 0x40, 0x6E};

const uint8_t RACEBOX_CHAR_GNSS_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                          0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                          0x04, 0x00, 0x40, 0x6E};

// BLE Services & Characteristics
BLEService rbService(RACEBOX_SERVICE_UUID);
BLECharacteristic rbTx(RACEBOX_CHAR_TX_UUID);
BLECharacteristic rbRx(RACEBOX_CHAR_RX_UUID);
BLECharacteristic rbGnss(RACEBOX_CHAR_GNSS_UUID);

// Device Info Service
BLEService disService(UUID16_SVC_DEVICE_INFORMATION);
BLECharacteristic disModel(UUID16_CHR_MODEL_NUMBER_STRING);
BLECharacteristic disSerial(UUID16_CHR_SERIAL_NUMBER_STRING);
BLECharacteristic disFirmware(UUID16_CHR_FIRMWARE_REVISION_STRING);
BLECharacteristic disHardware(UUID16_CHR_HARDWARE_REVISION_STRING);
BLECharacteristic disManuf(UUID16_CHR_MANUFACTURER_NAME_STRING);

bool deviceConnected = false;

// --- Packet Timing ---
unsigned long lastPacketSendTime = 0;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
const unsigned long GPS_RATE_REPORT_INTERVAL_MS = 5000;
unsigned int gnssUpdateCount = 0;

// --- Helper Functions ---
void writeLittleEndian(uint8_t *buffer, int offset, uint32_t value) {
  memcpy(buffer + offset, &value, 4);
}
void writeLittleEndian(uint8_t *buffer, int offset, int32_t value) {
  memcpy(buffer + offset, &value, 4);
}
void writeLittleEndian(uint8_t *buffer, int offset, uint16_t value) {
  memcpy(buffer + offset, &value, 2);
}
void writeLittleEndian(uint8_t *buffer, int offset, int16_t value) {
  memcpy(buffer + offset, &value, 2);
}
void writeLittleEndian(uint8_t *buffer, int offset, uint8_t value) {
  buffer[offset] = value;
}
void writeLittleEndian(uint8_t *buffer, int offset, int8_t value) {
  buffer[offset] = (uint8_t)value;
}

void calculateChecksum(uint8_t *payload, uint16_t len, uint8_t cls, uint8_t id,
                       uint8_t *ckA, uint8_t *ckB) {
  *ckA = *ckB = 0;
  *ckA += cls;
  *ckB += *ckA;
  *ckA += id;
  *ckB += *ckA;
  *ckA += len & 0xFF;
  *ckB += *ckA;
  *ckA += len >> 8;
  *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}
float getBatteryVoltage() {
  // 1. Enable the voltage divider
  digitalWrite(PIN_VBAT_ENABLE, LOW);
  // 2. Read the analog value
  // The XIAO divider is 1/2, and the internal ref is 3.6V (on nRF52)
  // Voltage = ADC_Result * (3.6 / 1024) * 2
  unsigned int adcCount = analogRead(PIN_VBAT);
  float voltage = adcCount * (3.6 / 1024.0) * 2.0;
  // digitalWrite(PIN_VBAT_ENABLE, HIGH);
  return voltage;
}

uint8_t getBatteryPercentage() {
  float v = getBatteryVoltage();
  if (v >= 4.2)
    return 100;
  if (v <= 3.5)
    return 0;
  // Simple linear mapping (4.2V - 3.5V)
  return (uint8_t)((v - 3.5) / (4.2 - 3.5) * 100.0);
}
bool isCharging() {
  return digitalRead(PIN_CHG) == LOW;
}

void resetGpsBaudRate() {
  Serial.println("Attempting to set Correct Baud Rate");
  Serial1.begin(FACTORY_GPS_BAUD); // XIAO uses Serial1 for hardware UART
  delay(500);

  if (!myGNSS.begin(Serial1)) {
    Serial.print("u-blox GNSS not detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud.");
    while (1)
      delay(100);
  } else {
    Serial.print("GNSS detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud!");
  }
  delay(500);

  Serial.print("Setting baud rate to ");
  Serial.print(GPS_BAUD);
  Serial.println("...");
  myGNSS.setSerialRate(GPS_BAUD);

  Serial1.flush();
  Serial1.end();
  delay(100);
  Serial1.begin(GPS_BAUD);
  delay(500);

  if (!myGNSS.begin(Serial1)) {
    Serial.println("GNSS lost after baud change.");
    while (1)
      delay(100);
  }

  Serial.println("GNSS re-connected at high baud! Saving config.");
  myGNSS.saveConfiguration();
}

// BLE Connection Callbacks
void connect_callback(uint16_t conn_handle) {
  deviceConnected = true;
  digitalWrite(OnboardledPin, LOW);
  Serial.println("✅ Client connected!");
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Connection(conn_handle)->requestMtuExchange(128);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  deviceConnected = false;
  Bluefruit.Advertising.setInterval(160, 244); // Slower = less power
  Serial.println("❌ BLE Client disconnected.");
}

void write_callback(uint16_t conn_handle, BLECharacteristic *chr, uint8_t *data,
                    uint16_t len) {
  Serial.print("📨 Received BLE command: ");
  for (int i = 0; i < len; i++)
    Serial.printf("0x%02X ", data[i]);
  Serial.println();
}

void setIMUForSleep() {
  IMU.settings.gyroEnabled = 0;
  IMU.settings.accelEnabled = 0;
  IMU.begin();

  // 52Hz, ±2g
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x30);

  // Enable tap detection
  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);

  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x8C);

  IMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x20);

  // Enable single+double tap
  IMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);

  // Low power accel
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);

  // Route to INT1
  IMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);

  // Enable wake pin
  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN_SENSE);
}

void enterDeepSleep() {
  Serial.println("💤 Entering Deep Sleep...");
  // XIAO RGB LEDs are Active Low, so HIGH = OFF
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(GPS_EN_PIN, LOW); // GPS OFF

  setIMUForSleep();
  delay(100); // Small delay for I2C to finish and IMU to settle
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  Serial.flush(); // Ensure serial message is sent before power cut
  NRF_POWER->SYSTEMOFF = 1;
}
void clearAllIMUInterrupts() {
  uint8_t dummy;
  IMU.readRegister(&dummy, 0x1B); // WAKE_UP_SRC
  IMU.readRegister(&dummy, 0x1C); // TAP_SRC
  IMU.readRegister(&dummy, 0x1D); // D6D_SRC
  IMU.readRegister(&dummy, 0x3B); // STATUS_REG (Check if data is pending)
  IMU.readRegister(&dummy, 0x53); // FUNC_SRC1 (Step counter/Tilt)
}

unsigned long lastActivityTime = 0; // Tracks last connected activity

void setup() {
  Serial.begin(115200);
  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  // battery Charging Rate

  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);
  pinMode(11, INPUT_PULLDOWN);
  // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT); // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(12);    // wireing_analog_nRF52.c:39
  lastActivityTime = 0;        // Start at 0 to track first connection

  digitalWrite(PIN_HICHG,
               LOW); // Set charge current to 100mA (LOW = 100mA, HIGH = 50mA)

  pinMode(OnboardledPin, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  // --- IMU Init ---
  if (IMU.begin() != 0) {
    Serial.println("❌ Failed to find LSM6DS3 chip");
  } else {
    Serial.println("✅ LSM6DS3 Found!");
  }

  // Init filter vars
  filtered_ax = IMU.readFloatAccelX();
  filtered_ay = IMU.readFloatAccelY();
  filtered_az = IMU.readFloatAccelZ();

  // --- GPS Init ---
  Serial1.begin(GPS_BAUD);
  if (!myGNSS.begin(Serial1)) {
    Serial.println("❌ GNSS not detected. Attempting to configure.");
    resetGpsBaudRate();
  }

  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
  myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE);

// --- GNSS Constellation Setup ---

// GPS
#ifdef ENABLE_GNSS_GPS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS)) {
    Serial.println("✅ GPS enabled.");
  } else {
    Serial.println("❌ Failed to enable GPS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GPS);
  Serial.println("🚫 GPS disabled.");
#endif

// Galileo
#ifdef ENABLE_GNSS_GALILEO
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO)) {
    Serial.println("✅ Galileo enabled.");
  } else {
    Serial.println("❌ Failed to enable Galileo.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
  Serial.println("🚫 Galileo disabled.");
#endif

// GLONASS
#ifdef ENABLE_GNSS_GLONASS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS)) {
    Serial.println("✅ GLONASS enabled.");
  } else {
    Serial.println("❌ Failed to enable GLONASS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
  Serial.println("🚫 GLONASS disabled.");
#endif

// BeiDou
#ifdef ENABLE_GNSS_BEIDOU
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU)) {
    Serial.println("✅ BEIDOU enabled.");
  } else {
    Serial.println("❌ Failed to enable BEIDOU.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
  Serial.println("🚫 BEIDOU disabled.");
#endif

// Optional: QZSS
#ifdef ENABLE_GNSS_QZSS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS)) {
    Serial.println("✅ QZSS enabled.");
  } else {
    Serial.println("❌ Failed to enable QZSS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS);
  Serial.println("🚫 QZSS disabled.");
#endif

// Optional: SBAS (satellite-based augmentation)
#ifdef ENABLE_GNSS_SBAS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS)) {
    Serial.println("✅ SBAS enabled.");
  } else {
    Serial.println("❌ Failed to enable SBAS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
  Serial.println("🚫 SBAS disabled.");
#endif

  // --- BLE PERFORMANCE CONFIGURATION ---
  // 1. Maximize Bandwidth (Vital for 25Hz)
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // 2. Init Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(deviceName.c_str());
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // 3. Request Fast Connection Interval (7.5ms - 15ms)
  // This forces the phone to listen much more frequently
  Bluefruit.Periph.setConnInterval(6, 12);

  // --- Service Setup ---
  disService.begin();
  disModel.setProperties(CHR_PROPS_READ);
  disModel.begin();
  disModel.write("RaceBox Mini");
  disSerial.setProperties(CHR_PROPS_READ);
  disSerial.begin();
  disSerial.write("0123456789");
  disFirmware.setProperties(CHR_PROPS_READ);
  disFirmware.begin();
  disFirmware.write("3.3");
  disManuf.setProperties(CHR_PROPS_READ);
  disManuf.begin();
  disManuf.write("RaceBox");

  rbService.begin();

  rbTx.setProperties(CHR_PROPS_NOTIFY);
  rbTx.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rbTx.setFixedLen(88);
  rbTx.begin();

  rbRx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rbRx.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rbRx.setWriteCallback(write_callback);
  rbRx.begin();

  rbGnss.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  rbGnss.begin();

  // Advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(rbService);
  Bluefruit.Advertising.addService(disService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 244); // Slower = less power
  Bluefruit.Advertising.setFastTimeout(30);
  lastActivityTime = 0;        // Start at 0 to track first connection
  Bluefruit.Advertising.start(0);
  Serial.println("📡 BLE Broadcast started.");
}

void loop() {
  myGNSS.checkUblox();
  // Battery Timing
  static unsigned long lastBatCheck = 0;
  static uint8_t currentBatPct = 100;
  // Accelrometer Timing
  static unsigned long lastAccelReadMs = 0;
  if (!deviceConnected) {
  // Sample at 1Hz instead of 100Hz
  if (millis() - lastAccelReadMs >= 1000) { // 1 second
    lastAccelReadMs = millis();

    // XIAO Sense IMU reads (LSM6DS3)
    // Note: LSM6DS3 returns G-Force (float) and Deg/Sec (float) directly
    float ax = IMU.readFloatAccelX();
    float ay = IMU.readFloatAccelY();
    float az = IMU.readFloatAccelZ();

    float gx = IMU.readFloatGyroX();
    float gy = IMU.readFloatGyroY();
    float gz = IMU.readFloatGyroZ();

    // EMA Filter
    filtered_ax = (accelAlpha * ax) + ((1.0 - accelAlpha) * filtered_ax);
    filtered_ay = (accelAlpha * ay) + ((1.0 - accelAlpha) * filtered_ay);
    filtered_az = (accelAlpha * az) + ((1.0 - accelAlpha) * filtered_az);

    filtered_gx = (gyroAlpha * gx) + ((1.0 - gyroAlpha) * filtered_gx);
    filtered_gy = (gyroAlpha * gy) + ((1.0 - gyroAlpha) * filtered_gy);
    filtered_gz = (gyroAlpha * gz) + ((1.0 - gyroAlpha) * filtered_gz);
  }
  } else {
    // Full 100Hz when connected
    if (millis() - lastAccelReadMs >= AccelSampleInterval) {
      lastAccelReadMs = millis();

    // XIAO Sense IMU reads (LSM6DS3)
    // Note: LSM6DS3 returns G-Force (float) and Deg/Sec (float) directly
    float ax = IMU.readFloatAccelX();
    float ay = IMU.readFloatAccelY();
    float az = IMU.readFloatAccelZ();

    float gx = IMU.readFloatGyroX();
    float gy = IMU.readFloatGyroY();
    float gz = IMU.readFloatGyroZ();

    // EMA Filter
    filtered_ax = (accelAlpha * ax) + ((1.0 - accelAlpha) * filtered_ax);
    filtered_ay = (accelAlpha * ay) + ((1.0 - accelAlpha) * filtered_ay);
    filtered_az = (accelAlpha * az) + ((1.0 - accelAlpha) * filtered_az);

    filtered_gx = (gyroAlpha * gx) + ((1.0 - gyroAlpha) * filtered_gx);
    filtered_gy = (gyroAlpha * gy) + ((1.0 - gyroAlpha) * filtered_gy);
    filtered_gz = (gyroAlpha * gz) + ((1.0 - gyroAlpha) * filtered_gz);
    }
  }

  // GNSS Data Handling
  if (myGNSS.getPVT()) {
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;

    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;

      if (deviceConnected && myGNSS.packetUBXNAVPVT != NULL) {
        gpsUpdateCount++;

        // --- IMU Unit Conversion for RaceBox Protocol ---
        // Protocol expects: Accel in milli-g, Gyro in centi-deg/s

        // LSM6DS3 provides 'g'. 1.0g * 1000 = 1000 mg
        int16_t gX = (int16_t)(filtered_ax * 1000.0);
        int16_t gY = (int16_t)(filtered_ay * 1000.0);
        int16_t gZ = (int16_t)(filtered_az * 1000.0);

        // LSM6DS3 provides 'dps'. 1.0 dps * 100 = 100 centi-dps
        int16_t rX = (int16_t)(filtered_gx * 100.0);
        int16_t rY = (int16_t)(filtered_gy * 100.0);
        int16_t rZ = (int16_t)(filtered_gz * 100.0);

        uint8_t payload[80] = {0};
        uint8_t packet[88] = {0};

        // --- Fill Payload (Same logic as original) ---
        writeLittleEndian(payload, 0, myGNSS.packetUBXNAVPVT->data.iTOW);
        writeLittleEndian(payload, 4, myGNSS.packetUBXNAVPVT->data.year);
        writeLittleEndian(payload, 6, myGNSS.packetUBXNAVPVT->data.month);
        writeLittleEndian(payload, 7, myGNSS.packetUBXNAVPVT->data.day);
        writeLittleEndian(payload, 8, myGNSS.packetUBXNAVPVT->data.hour);
        writeLittleEndian(payload, 9, myGNSS.packetUBXNAVPVT->data.min);
        writeLittleEndian(payload, 10, myGNSS.packetUBXNAVPVT->data.sec);

        uint8_t raceboxValidityFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate)
          raceboxValidityFlags |= (1 << 0);
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime)
          raceboxValidityFlags |= (1 << 1);
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved)
          raceboxValidityFlags |= (1 << 2);
        writeLittleEndian(payload, 11, raceboxValidityFlags);

        writeLittleEndian(payload, 12, myGNSS.packetUBXNAVPVT->data.tAcc);
        writeLittleEndian(payload, 16, myGNSS.packetUBXNAVPVT->data.nano);
        writeLittleEndian(payload, 20, myGNSS.packetUBXNAVPVT->data.fixType);

        uint8_t fixStatusFlagsRacebox = 0;
        if (myGNSS.packetUBXNAVPVT->data.fixType == 3)
          fixStatusFlagsRacebox |= (1 << 0);
        if (myGNSS.getHeadVehValid())
          fixStatusFlagsRacebox |= (1 << 5);
        writeLittleEndian(payload, 21, fixStatusFlagsRacebox);

        uint8_t raceboxDateTimeFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime)
          raceboxDateTimeFlags |= (1 << 5);
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate)
          raceboxDateTimeFlags |= (1 << 6);
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime &&
            myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved)
          raceboxDateTimeFlags |= (1 << 7);
        writeLittleEndian(payload, 22, raceboxDateTimeFlags);

        writeLittleEndian(payload, 23, myGNSS.packetUBXNAVPVT->data.numSV);
        writeLittleEndian(payload, 24, myGNSS.packetUBXNAVPVT->data.lon);
        writeLittleEndian(payload, 28, myGNSS.packetUBXNAVPVT->data.lat);
        writeLittleEndian(payload, 32, myGNSS.packetUBXNAVPVT->data.height);
        writeLittleEndian(payload, 36, myGNSS.packetUBXNAVPVT->data.hMSL);
        writeLittleEndian(payload, 40, myGNSS.packetUBXNAVPVT->data.hAcc);
        writeLittleEndian(payload, 44, myGNSS.packetUBXNAVPVT->data.vAcc);
        writeLittleEndian(payload, 48, myGNSS.packetUBXNAVPVT->data.gSpeed);
        writeLittleEndian(payload, 52, myGNSS.packetUBXNAVPVT->data.headMot);
        writeLittleEndian(payload, 56, myGNSS.packetUBXNAVPVT->data.sAcc);
        writeLittleEndian(payload, 60, myGNSS.packetUBXNAVPVT->data.headAcc);
        writeLittleEndian(payload, 64, myGNSS.packetUBXNAVPVT->data.pDOP);

        uint8_t latLonFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.fixType < 2)
          latLonFlags |= (1 << 0);
        writeLittleEndian(payload, 66, latLonFlags);

        if (millis() - lastBatCheck > 5000) { // Check every 5 seconds
          currentBatPct = getBatteryPercentage();
          lastBatCheck = millis();
        }
        uint8_t batteryField =
            currentBatPct & 0x7F; // Ensure percentage only uses 7 bits (0-127)

        if (isCharging()) {
          batteryField |= 0x80; // Set the most significant bit to 1
        }

        writeLittleEndian(payload, 67, batteryField); // Battery 100%

        writeLittleEndian(payload, 68, gX);
        writeLittleEndian(payload, 70, gY);
        writeLittleEndian(payload, 72, gZ);
        writeLittleEndian(payload, 74, rX);
        writeLittleEndian(payload, 76, rY);
        writeLittleEndian(payload, 78, rZ);

        packet[0] = 0xB5;
        packet[1] = 0x62;
        packet[2] = 0xFF;
        packet[3] = 0x01;
        packet[4] = 80;
        packet[5] = 0;
        memcpy(packet + 6, payload, 80);
        uint8_t ckA, ckB;
        calculateChecksum(payload, 80, 0xFF, 0x01, &ckA, &ckB);
        packet[86] = ckA;
        packet[87] = ckB;

        rbTx.notify(packet, 88);
      }
    }

    // Report Rates
    // Report Rates and Battery Status
    const unsigned long now = millis();
    if ((now - lastGpsRateCheckTime) >= GPS_RATE_REPORT_INTERVAL_MS) {
      float bleRate = gpsUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
      float gnssRate = gnssUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
      currentBatPct = getBatteryPercentage();
      uint8_t batteryField =
          currentBatPct & 0x7F; // Ensure percentage only uses 7 bits (0-127)

      if (isCharging()) {
        batteryField |= 0x80; // Set the most significant bit to 1
      }
      // --- Battery Debug Decoding ---
      // Extract percentage by clearing the top bit (0x7F = 01111111)
      uint8_t displayPct = batteryField & 0x7F;
      // Check if the top bit is 1 (0x80 = 10000000)
      bool displayCharging = (batteryField & 0x80) != 0;

      uint8_t sats = 0;
      uint8_t fix = 0;
      if (myGNSS.packetUBXNAVPVT != NULL) {
        sats = myGNSS.packetUBXNAVPVT->data.numSV;
        fix = myGNSS.packetUBXNAVPVT->data.fixType;
      }

      // Comprehensive Debug Line
      Serial.printf("--------------------------------------------------\n");
      Serial.printf("STATUS | BLE: %.2f Hz | GNSS: %.2f Hz\n", bleRate,
                    gnssRate);
      Serial.printf("GPS    | SVs: %u | Fix: %u\n", sats, fix);
      Serial.printf("POWER  | Bat: %d%% | Charging: %s (Raw: 0x%02X)\n",
                    displayPct, displayCharging ? "YES ⚡" : "NO 🔋",
                    batteryField);
      // --- Calculate Time Remaining ---
      long secondsUntilSleep = 0;

      if (!deviceConnected) {
        if (isCharging()){
          Serial.printf("SLEEP  | Disabled , Charging\n");
        } else if (lastActivityTime == 0) {
          // Wakeup grace period: never connected since boot/wake
          secondsUntilSleep = (60000 - (now - lastActivityTime)) / 1000;
          Serial.printf("SLEEP  | Mode: Wakeup Grace | Timeout in: %lds\n",
                        secondsUntilSleep > 0 ? secondsUntilSleep : 0);
        } else if (lastActivityTime > 0) {
          // Idle timeout: was connected, now disconnected
          secondsUntilSleep = (SLEEP_TIMEOUT - (now - lastActivityTime)) / 1000;
          Serial.printf("SLEEP  | Mode: Idle Timeout | Timeout in: %lds\n",
                        secondsUntilSleep > 0 ? secondsUntilSleep : 0);
        }  
        
      }
      else {
        Serial.printf("SLEEP  | Status: Connected (Timer Paused)\n");
      }
      Serial.printf("--------------------------------------------------\n");
      gpsUpdateCount = 0;
      gnssUpdateCount = 0;
      lastGpsRateCheckTime = now;
    }
  }
  const unsigned long now = millis();
  bool currentlyCharging = isCharging();
  if (lastChargingState == true && currentlyCharging == false) {
    // Just unplugged! Reset activity to now to delay the SLEEP_TIMEOUT
    lastActivityTime = now;
    lastChargingState = false;
  }
  else if (currentlyCharging) {
    lastChargingState = true; 
  }

  // Update lastActivityTime while connected
  if (deviceConnected) {
    lastActivityTime = now;
  }

  // Sleep Logic:
  // - If lastActivityTime = 0, 60s grace period
  // - If previously connected (lastActivityTime > 0): SLEEP_TIMEOUT after
  // disconnect
  // Dont sleep if connected or charging
  if (!currentlyCharging){
    if (!deviceConnected) {
      if (lastActivityTime == 0) {
        // Shake-to-wake grace period: sleep if no connection within 60 seconds
        if (now - lastActivityTime > 60000) {
          enterDeepSleep();
        }
      } else if (lastActivityTime > 0) {
        // Idle timeout: sleep if disconnected for SLEEP_TIMEOUT
        if (now - lastActivityTime > SLEEP_TIMEOUT) {
          enterDeepSleep();
        }
      }
    }
  }
}
