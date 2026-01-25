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

// --- Power Management Configuration ---
#define GPS_EN_PIN D1
const unsigned long GPS_HOT_TIMEOUT =
    900000; // 15 minutes - keep GPS hot after disconnect
const unsigned long DEEP_SLEEP_TIMEOUT =
    604800000; // 7 days - long term storage safety net
const bool ENABLE_DEEP_SLEEP = false;
#define FAST_ADV_INTERVAL 160 // 100ms
#define ECO_ADV_INTERVAL 1600 // 1000ms

// --- Battery Monitoring ---
#define PIN_VBAT (32)        // D32 battery voltage
#define PIN_VBAT_ENABLE (14) // D14 LOW:read enable
#define PIN_HICHG (22)       // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG (23)         // D23 charge indicator LOW:charge HIGH:no charge

// --- GNSS Constellations ---
#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
#define ENABLE_GNSS_GLONASS
#define ENABLE_GNSS_BEIDOU
// #define ENABLE_GNSS_SBAS
// #define ENABLE_GNSS_QZSS

const char *rawDeviceName = "RaceBox Mini 0123456789";
const String deviceName = rawDeviceName;

// LED Configuration (Blue LED on XIAO)
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
bool gpsEnabled = false;
bool imuEnabled = false;
bool pendingConfig = false; // Flag to request sensor refresh in main loop
unsigned long lastDisconnectTime = 0;
unsigned long lastActivityTime = 0; // Tracks last activity for deep sleep

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
bool isCharging() { return digitalRead(PIN_CHG) == LOW; }

// --- Sensor Management ---
bool configureGPS() {
  if (pendingConfig) {
    Serial.println("⚙️ Syncing GPS Settings...");

    // 1. Ensure Serial1 is ready
    Serial1.begin(GPS_BAUD);

    // 2. Try to detect module with retries
    bool detected = false;
    for (int i = 0; i < 3; i++) {
      if (myGNSS.begin(Serial1)) {
        detected = true;
        break;
      }
      delay(20);
    }

    // 3. Fallback to 9600 to upgrade baud
    if (!detected) {
      Serial.println("⚠️ GPS not at 115200, checking 9600...");
      Serial1.begin(FACTORY_GPS_BAUD);
      delay(50);
      if (myGNSS.begin(Serial1)) {
        myGNSS.setSerialRate(GPS_BAUD);
        delay(100);
        Serial1.begin(GPS_BAUD);
        if (!myGNSS.begin(Serial1))
          return false;
      } else {
        return false;
      }
    }

    myGNSS.setPortOutput(COM_PORT_UART1, COM_TYPE_UBX);
    myGNSS.setAutoPVT(true);
    myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE);

#ifdef ENABLE_GNSS_GPS
    myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
#endif
#ifdef ENABLE_GNSS_GALILEO
    myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
#endif
#ifdef ENABLE_GNSS_GLONASS
    myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
#endif
#ifdef ENABLE_GNSS_BEIDOU
    myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
#endif

    pendingConfig = false;
    Serial.println("✅ Configuration complete.");
    return true;
  }
  return false; // Already synced
}

void enableGPS() {
  if (!gpsEnabled) {
    Serial.println("🛰️ enableGPS() starting...");
    digitalWrite(GPS_EN_PIN, HIGH);
    gpsEnabled = true;
    delay(100);

    if (!deviceConnected) {
      Bluefruit.Advertising.stop();
      Bluefruit.Advertising.setInterval(FAST_ADV_INTERVAL,
                                        FAST_ADV_INTERVAL + 100);
      Bluefruit.Advertising.start(0);
    }
    Serial.println("✅ enableGPS() finished.");
  }
}

void disableGPS() {
  Serial.println("🛰️ disableGPS() starting...");
  pendingConfig = true;
  digitalWrite(GPS_EN_PIN, LOW);
  gpsEnabled = false;
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  if (!deviceConnected) {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.setInterval(ECO_ADV_INTERVAL, ECO_ADV_INTERVAL + 200);
    Bluefruit.Advertising.start(0);
  }
  Serial.println("✅ disableGPS() finished.");
}

void enableIMU() {
  if (!imuEnabled) {
    Serial.println("📊 enableIMU() starting...");
    IMU.settings.gyroEnabled = 1;
    IMU.settings.accelEnabled = 1;
    IMU.settings.accelRange = 2;
    IMU.settings.gyroRange = 2000;

    if (IMU.begin() != 0) {
      Serial.println("⚠️ IMU Enable Failed");
      return;
    }

    filtered_ax = IMU.readFloatAccelX();
    filtered_ay = IMU.readFloatAccelY();
    filtered_az = IMU.readFloatAccelZ();
    filtered_gx = IMU.readFloatGyroX();
    filtered_gy = IMU.readFloatGyroY();
    filtered_gz = IMU.readFloatGyroZ();

    imuEnabled = true;
    Serial.println("✅ enableIMU() finished.");
  }
}

void disableIMU() {
  if (imuEnabled) {
    Serial.println("📊 disableIMU() starting...");
    IMU.settings.gyroEnabled = 0;
    IMU.settings.accelEnabled = 0;
    IMU.begin();
    imuEnabled = false;
    Serial.println("✅ disableIMU() finished.");
  }
}

// --- LED Fix Status ---
void updateFixLEDs(uint8_t fixType) {
  if (!gpsEnabled) {
    digitalWrite(LED_RED, HIGH);   // OFF
    digitalWrite(LED_GREEN, HIGH); // OFF
    return;
  }

  switch (fixType) {
  case 3:                         // 3D Fix
  case 4:                         // GNSS + Dead Reckoning
    digitalWrite(LED_RED, HIGH);  // Red OFF
    digitalWrite(LED_GREEN, LOW); // Green ON
    break;
  case 1:                         // Dead Reckoning only
  case 2:                         // 2D Fix
    digitalWrite(LED_RED, LOW);   // Red ON
    digitalWrite(LED_GREEN, LOW); // Green ON -> Orange/Yellow
    break;
  default:                         // No fix or Time only
    digitalWrite(LED_RED, LOW);    // Red ON
    digitalWrite(LED_GREEN, HIGH); // Green OFF
    break;
  }
}

// BLE Connection Callbacks
void connect_callback(uint16_t conn_handle) {
  deviceConnected = true;
  // Note: pendingConfig is NOT set here to allow "Instant-On" reconnects
  digitalWrite(OnboardledPin, LOW); // Solid Blue ON when connected
  Serial.println("✅ Client connected!");

  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Connection(conn_handle)->requestMtuExchange(128);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  deviceConnected = false;
  lastDisconnectTime = millis(); // Start GPS hot timeout timer immediately

  // Turn off Blue LED immediately on disconnect
  digitalWrite(OnboardledPin, HIGH);

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.setInterval(ECO_ADV_INTERVAL, ECO_ADV_INTERVAL + 200);
  Bluefruit.Advertising.start(0);
  Serial.println("❌ BLE Client disconnected.");
  Serial.println("📡 BLE advertising restarted (ECO).");
  Serial.println("🛰️ GPS staying hot for 15 minutes...");
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
  Serial.println("💤 Entering Deep Sleep (Shake to Wake)...");

  // Turn off all LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  // Ensure GPS is off
  disableGPS();

  // Configure Triggers for Wake-up
  setIMUForSleep(); // Trigger 1: Shake (IMU INT pin)
  pinMode(PIN_CHG,
          INPUT_PULLUP_SENSE); // Trigger 2: Plug-in (Charge pin goes LOW)

  delay(100); // Small delay for I2C to finish and IMU to settle
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  Serial.flush(); // Ensure serial message is sent before power cut
  Bluefruit.autoConnLed(false);
  NRF_POWER->SYSTEMOFF = 1;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for Serial to stabilize
  Serial.println("\n\n🚀 SYSTEM RESTART / BOOT");
  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH); // Power on for initialization
  pinMode(11, INPUT_PULLDOWN);

  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);

  Wire.setClock(400000); // Fast I2C for IMU

  analogReference(AR_DEFAULT);
  analogReadResolution(12);

  gpsEnabled = false; // MUST BE FALSE for enableGPS() to work in setup
  lastActivityTime = millis();
  lastDisconnectTime = 0;

  digitalWrite(PIN_HICHG, LOW);

  pinMode(OnboardledPin, OUTPUT);
  digitalWrite(OnboardledPin, HIGH); // Turn off Blue LED initially
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  // --- IMU Init ---
  if (IMU.begin() != 0) {
    Serial.println("❌ Failed to find LSM6DS3 chip");
  } else {
    Serial.println("✅ LSM6DS3 Found!");
    imuEnabled = true;
    disableIMU(); // Power down after successful init
  }

  // Init filter vars
  filtered_ax = IMU.readFloatAccelX();
  filtered_ay = IMU.readFloatAccelY();
  filtered_az = IMU.readFloatAccelZ();

  // --- BLE PERFORMANCE CONFIGURATION ---
  // 1. Maximize Bandwidth (Vital for 25Hz)
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // 2. Init Bluefruit
  Bluefruit.begin();
  Bluefruit.autoConnLed(false); // SILENCE THE BLINK FOREVER
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
  Bluefruit.Advertising.restartOnDisconnect(
      false); // We'll handle this manually
  Bluefruit.Advertising.setInterval(ECO_ADV_INTERVAL,
                                    ECO_ADV_INTERVAL + 200); // Start in Eco
  Bluefruit.Advertising.setFastTimeout(30);
  lastActivityTime = 0; // Start at 0 to track first connection
  Bluefruit.Advertising.start(0);
  Serial.println("📡 BLE Broadcast started.");

  // Power down sensors until first connection
  disableGPS();
  disableIMU();
}

void loop() {
  static unsigned long loopCount = 0;
  if (++loopCount % 5000 == 0)
    Serial.printf("💓 Loop Heartbeat: %lu\n", millis());

  const unsigned long now = millis();

  // --- Background Processing ---
  if (gpsEnabled)
    myGNSS.checkUblox();

  // Battery Timing
  static unsigned long lastBatCheck = 0;
  static uint8_t currentBatPct = 100;

  // Accelerometer Timing
  static unsigned long lastAccelReadMs = 0;

  // Performance logic: only read IMU if enabled (connected)
  if (imuEnabled) {
    if (millis() - lastAccelReadMs >= AccelSampleInterval) {
      lastAccelReadMs = millis();

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

  // GNSS Data Handling - only if enabled
  bool newPVT = false;
  if (gpsEnabled && myGNSS.getPVT()) {
    newPVT = true;
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;

    // Update LEDs based on current fix type
    updateFixLEDs(myGNSS.packetUBXNAVPVT->data.fixType);

    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;

      if (deviceConnected && myGNSS.packetUBXNAVPVT != NULL) {
        // --- IMU Unit Conversion for RaceBox Protocol ---
        int16_t gX = (int16_t)(filtered_ax * 1000.0);
        int16_t gY = (int16_t)(filtered_ay * 1000.0);
        int16_t gZ = (int16_t)(filtered_az * 1000.0);
        int16_t rX = (int16_t)(filtered_gx * 100.0);
        int16_t rY = (int16_t)(filtered_gy * 100.0);
        int16_t rZ = (int16_t)(filtered_gz * 100.0);

        uint8_t payload[80] = {0};
        uint8_t packet[88] = {0};

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

        if (millis() - lastBatCheck > 5000) {
          currentBatPct = getBatteryPercentage();
          lastBatCheck = millis();
        }
        uint8_t batteryField = currentBatPct & 0x7F;
        if (isCharging())
          batteryField |= 0x80;

        writeLittleEndian(payload, 67, batteryField);
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

        if (rbTx.notify(packet, 88)) {
          gpsUpdateCount++;
        }
      }
    }
  }

  // --- Fast Re-Sync logic ---
  // If we are connected but don't get data from u-blox, it might be in a bad
  // state
  static unsigned long lastUbloxDataTime = 0;
  if (gpsEnabled) {
    if (newPVT)
      lastUbloxDataTime = millis();
    if (deviceConnected && (millis() - lastUbloxDataTime > 2000)) {
      myGNSS.checkUblox(); // Force background processing
    }
  }

  // --- Stability Delay ---
  delay(1);

  // Debug Reporting
  if ((now - lastGpsRateCheckTime) >= GPS_RATE_REPORT_INTERVAL_MS) {
    float bleRate = gpsUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
    float gnssRate = gnssUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
    currentBatPct = getBatteryPercentage();
    bool charging = isCharging();

    Serial.printf("--------------------------------------------------\n");
    Serial.printf("POWER  | Bat: %d%% | Charging: %s | State: %s\n",
                  currentBatPct, charging ? "YES ⚡" : "NO 🔋",
                  deviceConnected ? "CONNECTED" : "IDLE");
    Serial.printf("SENSORS| GPS: %s | IMU: %s\n", gpsEnabled ? "ON" : "OFF",
                  imuEnabled ? "ON" : "OFF");

    if (gpsEnabled) {
      uint8_t sats = 0, fix = 0;
      if (myGNSS.packetUBXNAVPVT != NULL) {
        sats = myGNSS.packetUBXNAVPVT->data.numSV;
        fix = myGNSS.packetUBXNAVPVT->data.fixType;
      }
      Serial.printf(
          "GNSS   | Fast: %.2f Hz | Loop: %.2f Hz | SVs: %u | Fix: %u\n",
          bleRate, gnssRate, sats, fix);
    }

    if (!deviceConnected && gpsEnabled && !charging) {
      long hotRem = (GPS_HOT_TIMEOUT - (now - lastDisconnectTime)) / 1000;
      Serial.printf("TIMER  | GPS Hot Timeout in: %lds\n",
                    hotRem > 0 ? hotRem : 0);
    }

    if (ENABLE_DEEP_SLEEP && !deviceConnected && !charging) {
      long sleepRem = (DEEP_SLEEP_TIMEOUT - (now - lastActivityTime)) / 1000;
      Serial.printf("TIMER  | Deep Sleep in: %lds\n",
                    sleepRem > 0 ? sleepRem : 0);
    }
    Serial.printf("--------------------------------------------------\n");

    gpsUpdateCount = 0;
    gnssUpdateCount = 0;
    lastGpsRateCheckTime = now;
  }

  // --- Power Management ---
  bool currentlyCharging = isCharging();

  if (deviceConnected || currentlyCharging) {
    lastActivityTime = millis();
    lastDisconnectTime = millis(); // Keep timer fresh while in use

    if (!gpsEnabled)
      enableGPS();
    if (!imuEnabled)
      enableIMU();

    // Always attempt sync if pending (only runs once per power-up)
    if (configureGPS()) {
      Serial.println("🔄 GPS Settings synced and ready.");
    }
  } else {
    // DISCONNECT CASE: Disable IMU immediately to save power (Thread-safe)
    if (imuEnabled)
      disableIMU();
  }

  // GPS Hot Timeout logic
  if (!deviceConnected && gpsEnabled && !currentlyCharging) {
    // Always use fresh millis() for subtraction to prevent race conditions
    if (millis() - lastDisconnectTime > GPS_HOT_TIMEOUT) {
      Serial.printf("⏰ GPS Hot Timeout reached (%lu ms). Powering down.\n",
                    GPS_HOT_TIMEOUT);
      disableGPS();
    }
  }

  // Optional Deep Sleep logic
  if (ENABLE_DEEP_SLEEP && !deviceConnected && !currentlyCharging) {
    if (now - lastActivityTime > DEEP_SLEEP_TIMEOUT) {
      enterDeepSleep();
    }
  }

  // --- Smart Recovery Watchdog ---
  static unsigned long lastValidData = 0;
  static unsigned long lastConnectionEvent = 0;
  static bool lastDeviceConnected = false;

  // Track connection state changes
  if (deviceConnected && !lastDeviceConnected) {
    lastConnectionEvent = millis();
    lastValidData = millis(); // Reset data timer on new connection
  }
  lastDeviceConnected = deviceConnected;

  if (gpsEnabled && newPVT)
    lastValidData = millis();

  if (deviceConnected && !pendingConfig) {
    unsigned long timeSinceLastData = millis() - lastValidData;
    unsigned long timeSinceConnected = millis() - lastConnectionEvent;

    // 1. Initial Check: If we just connected but see NO data for 1.2s, force
    // sync
    if (timeSinceConnected > 1200 && timeSinceLastData > 1000) {
      Serial.println("⚠️ Reconnection flow stalled (1s), forcing sync...");
      pendingConfig = true;
      lastValidData = millis(); // Prevent spamming
    }
    // 2. Steady State: If data ever stops for 3s, force re-sync
    else if (timeSinceLastData > 3000) {
      Serial.println("⚠️ GPS data stream stalled (3s), recovering...");
      pendingConfig = true;
      lastValidData = millis(); // Prevent spamming
    }
  }

  // Reset charging state tracker
  if (lastChargingState == true && currentlyCharging == false) {
    lastActivityTime = millis();
    if (!deviceConnected)
      lastDisconnectTime = millis();
  }
  lastChargingState = currentlyCharging;
}
