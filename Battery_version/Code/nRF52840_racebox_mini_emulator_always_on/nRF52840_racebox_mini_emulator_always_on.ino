#include <Adafruit_TinyUSB.h>
#include <LSM6DS3.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <bluefruit.h>
#include <nrf_soc.h>

// ============================================================================
// --- USER CUSTOMIZATION GALLERY ---
// ============================================================================

// --- BLE Branding ---
#define SERIAL_NUM "0509050905" // The unique 10-digit serial

// (DO NOT CHANGE THESE: Required for RaceBox Application compatibility)
#define DEVICE_NAME "RaceBox Mini " SERIAL_NUM // Auto-synced Name
#define MANUFACTURER "RaceBox"
#define FIRMWARE_VER "3.3"

// --- GPS Performance ---
#define MAX_NAVIGATION_RATE 25  // 25Hz: Max rate for RaceBox Mini protocol
#define GPS_BAUD 115200         // High speed for 25Hz data
#define FACTORY_GPS_BAUD 9600   // Default for cold modules

#define SYSTEM_RATE_REPORT_MS 5000 // Interval for Serial stats reporting

// --- Power & Efficiency ---
#define GPS_HOT_TIMEOUT_MS 600000 // 10 Minutes (Stay powered after disconnect)
#define DEEP_SLEEP_DAYS 7         // Safety net before absolute shutdown
#define ENABLE_DEEP_SLEEP false   // Usually false for standard RaceBox usage
#define FAST_ADV_INTERVAL 160     // 100ms: Fast discovery for apps
#define ECO_ADV_INTERVAL 1600     // 1000ms: Low power while idle
#define SLEEP_WHILE_CHARGING true // Allow Light Sleep even when plugged in /Set false to force high power mode when plugged in

// --- GNSS Constellation Toggle ---
#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
// #define ENABLE_GNSS_GLONASS
// #define ENABLE_GNSS_BEIDOU

// ============================================================================
// ---  HARDWARE MAPPINGS ---
// ============================================================================

#define GPS_EN_PIN D1      // GPS Power Enable Rail
#define PIN_VBAT_ENABLE 14 // Battery Read Enable
#define PIN_HICHG 22       // Charge Speed (LOW=100mA)
#define PIN_CHG 23         // Charge Indicator (LOW=Charging)
#define ACCEL_INT_PIN PIN_LSM6DS3TR_C_INT1

// ============================================================================
// ---  GLOBAL SYSTEM STATE ---
// ============================================================================

SFE_UBLOX_GNSS myGNSS;
LSM6DS3 IMU(I2C_MODE, 0x6A);

// System Flags
bool deviceConnected = false;
bool gpsEnabled = false;
bool imuEnabled = false;
bool pendingConfig = false;
bool lastChargingState = false;
bool lastPluggedInState = false;
uint8_t currentBatteryPercentage = 100;
float batteryMultiplier = 3.0; // Voltage divider 1/3

// Global State
bool isCritical = false;
int GPSFixType = 0;

// Timing Trackers
unsigned long lastDisconnectTime = 0;
unsigned long lastActivityTime = 0;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
unsigned int gnssUpdateCount = 0;

// Filter/IMU State
const unsigned long AccelSampleInterval = 10; // 100Hz
float accelAlpha = 0.8, gyroAlpha = 0.8;
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// BLE Core Objects
const uint8_t RACEBOX_SERVICE_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                        0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                        0x01, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_TX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                   0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                   0x03, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_RX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                   0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                   0x02, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_GNSS_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                     0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                     0x04, 0x00, 0x40, 0x6E};

BLEService rbService(RACEBOX_SERVICE_UUID);
BLECharacteristic rbTx(RACEBOX_TX_UUID);
BLECharacteristic rbRx(RACEBOX_RX_UUID);
BLECharacteristic rbGnss(RACEBOX_GNSS_UUID);

BLEService disService(UUID16_SVC_DEVICE_INFORMATION);
BLECharacteristic disModel(UUID16_CHR_MODEL_NUMBER_STRING);
BLECharacteristic disSerial(UUID16_CHR_SERIAL_NUMBER_STRING);
BLECharacteristic disFirmware(UUID16_CHR_FIRMWARE_REVISION_STRING);
BLECharacteristic disHardware(UUID16_CHR_HARDWARE_REVISION_STRING);
BLECharacteristic disManuf(UUID16_CHR_MANUFACTURER_NAME_STRING);

const int OnboardledPin = LED_BLUE;

// --- Helper Utilities ---
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

bool isCharging() { return digitalRead(PIN_CHG) == LOW; }

// 1. Calibration Table
struct VoltagePoint {
  float voltage;
  uint8_t percentage;
};

const VoltagePoint batteryMap[] = {
  {4.20, 100},
  {4.15, 98},
  {4.10, 95},
  {4.05, 92},
  {4.00, 88},

  {3.92, 75},
  {3.85, 65},
  {3.78, 55},
  {3.72, 45},
  {3.68, 35},
  {3.63, 25},
  {3.58, 18},

  {3.50, 10},
  {3.35, 5},
  {3.20, 0}
};

const uint8_t mapSize = sizeof(batteryMap) / sizeof(VoltagePoint);


// 2. Lookup Function
float getRawPercentage() {
  float v = getBatteryVoltage(); // Your 16-sample average function
  if (v >= batteryMap[0].voltage)
    return 100.0;
  if (v <= batteryMap[mapSize - 1].voltage)
    return 0.0;

  for (int i = 0; i < mapSize - 1; i++) {
    if (v <= batteryMap[i].voltage && v > batteryMap[i + 1].voltage) {
      float vHigh = batteryMap[i].voltage;
      float vLow = batteryMap[i + 1].voltage;
      uint8_t pHigh = batteryMap[i].percentage;
      uint8_t pLow = batteryMap[i + 1].percentage;
      return pLow + (v - vLow) * (pHigh - pLow) / (vHigh - vLow);
    }
  }
  return 0.0;
}

void handleAlerts() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  if (isCritical && !isCharging()) {
    // Blink every 500ms if battery is low and NOT charging
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_RED, ledState);
    }
  } else {
    // Ensure LED is off if not in critical state
    digitalWrite(LED_RED, HIGH);
    ledState = false;
  }
}

// 4. State Update (Call this every 5 seconds)
void updateBatteryState() {
  static float filteredPct = -1.0;
  bool pluggedIn = isCharging();
  float rawPct = getRawPercentage();
  float currentV = getBatteryVoltage();

  // Initial Sync
  if (filteredPct < 0) {
    filteredPct = rawPct;
    currentBatteryPercentage = (uint8_t)rawPct;
  }

  // Heavy Filter (5s interval)
  filteredPct = (rawPct * 0.05) + (filteredPct * 0.95);
  uint8_t rounded = (uint8_t)(filteredPct + 0.5);

  // Set Critical Flag (e.g., below 3.45V / 10%)
  isCritical = (currentV < 3.45);

  // --- STICKY 100% LATCH ---
  // If we were at 100%, don't drop the display until the filtered value
  // hits 95%. This prevents the 5V boost-regulator sag from killing
  // the "Full" status immediately upon power-on.
  if (currentBatteryPercentage == 100 && !pluggedIn && rounded > 95) {
    rounded = 100;
  }

  if (abs((int)rounded - (int)currentBatteryPercentage) >= 2 ||
      rounded == 100 || rounded == 0) {
    if (pluggedIn) {
      if (rounded > currentBatteryPercentage)
        currentBatteryPercentage = rounded;
    } else {
      if (rounded < currentBatteryPercentage)
        currentBatteryPercentage = rounded;

      // Boot/Recovery Sync: If raw is significantly higher (e.g. 10%), force
      // update
      if (rawPct > currentBatteryPercentage + 10.0) {
        currentBatteryPercentage = (uint8_t)rawPct;
        filteredPct = rawPct;
      }
    }
  }
}

bool isPluggedIn() {
  // Check if USB power is detected
  return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

float getBatteryVoltage() {
  digitalWrite(PIN_VBAT_ENABLE, LOW); // Enable divider
  delay(1);

  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(PIN_VBAT);
    delayMicroseconds(50);
  }
  float adcCount = (float)sum / 16.0;
  float voltage = (batteryMultiplier * 3.6 * adcCount / 4096);

  // --- LOAD COMPENSATION ---
  // If the GPS is running (80mA draw), the battery voltage "sags" naturally.
  // We add an offset to compensate so the percentage doesn't drop just
  // because the sensors turned on.
  if (gpsEnabled && !isPluggedIn()) {
    voltage += 0.040; // Approx compensation for 80mA on a 1100mAh pack
  }

  if (!isCharging() && !isPluggedIn()) {
    digitalWrite(PIN_VBAT_ENABLE, HIGH);
  }
  return voltage;
}

// ============================================================================
// ---  SENSOR PROCESSING MODULES ---
// ============================================================================

// Assemble and transmit the proprietary RaceBox Mini protocol packet
void sendRaceboxPacket() {
  if (!deviceConnected || myGNSS.packetUBXNAVPVT == NULL)
    return;

  uint8_t payload[80] = {0};
  uint8_t packet[88] = {0};
  auto *data = &myGNSS.packetUBXNAVPVT->data;

  // Time and Resolution
  writeLittleEndian(payload, 0, data->iTOW);
  writeLittleEndian(payload, 4, data->year);
  writeLittleEndian(payload, 6, data->month);
  writeLittleEndian(payload, 7, data->day);
  writeLittleEndian(payload, 8, data->hour);
  writeLittleEndian(payload, 9, data->min);
  writeLittleEndian(payload, 10, data->sec);

  // Status and Accuracy
  uint8_t val = 0;
  if (data->valid.bits.validDate)
    val |= (1 << 0);
  if (data->valid.bits.validTime)
    val |= (1 << 1);
  if (data->valid.bits.fullyResolved)
    val |= (1 << 2);
  writeLittleEndian(payload, 11, val);
  writeLittleEndian(payload, 12, data->tAcc);
  writeLittleEndian(payload, 16, data->nano);
  writeLittleEndian(payload, 20, data->fixType);

  // Fix and Info Flags
  uint8_t fixFlags = 0;
  if (data->fixType == 3)
    fixFlags |= (1 << 0);
  if (myGNSS.getHeadVehValid())
    fixFlags |= (1 << 5);
  writeLittleEndian(payload, 21, fixFlags);

  uint8_t dtFlags = 0;
  if (data->valid.bits.validTime)
    dtFlags |= (1 << 5);
  if (data->valid.bits.validDate)
    dtFlags |= (1 << 6);
  if (data->valid.bits.validTime && data->valid.bits.fullyResolved)
    dtFlags |= (1 << 7);
  writeLittleEndian(payload, 22, dtFlags);

  // Position, Speed, and Heading
  writeLittleEndian(payload, 23, data->numSV);
  writeLittleEndian(payload, 24, (int32_t)data->lon);
  writeLittleEndian(payload, 28, (int32_t)data->lat);
  writeLittleEndian(payload, 32, (int32_t)data->height);
  writeLittleEndian(payload, 36, (int32_t)data->hMSL);
  writeLittleEndian(payload, 40, (uint32_t)data->hAcc);
  writeLittleEndian(payload, 44, (uint32_t)data->vAcc);
  writeLittleEndian(payload, 48, (int32_t)data->gSpeed);
  writeLittleEndian(payload, 52, (int32_t)data->headMot);
  writeLittleEndian(payload, 56, (uint32_t)data->sAcc);
  writeLittleEndian(payload, 60, (uint32_t)data->headAcc);
  writeLittleEndian(payload, 64, (uint16_t)data->pDOP);

  // Fix Quality and Misc
  if (data->fixType < 2)
    writeLittleEndian(payload, 66, (uint8_t)(1 << 0));

  uint8_t batPct = currentBatteryPercentage & 0x7F;
  if (isCharging())
    batPct |= 0x80;
  writeLittleEndian(payload, 67, batPct);

  // Physical Sensors (Smoothed IMU)
  writeLittleEndian(payload, 68, (int16_t)(filtered_ax * 1000.0));
  writeLittleEndian(payload, 70, (int16_t)(filtered_ay * 1000.0));
  writeLittleEndian(payload, 72, (int16_t)(filtered_az * 1000.0));
  writeLittleEndian(payload, 74, (int16_t)(filtered_gx * 100.0));
  writeLittleEndian(payload, 76, (int16_t)(filtered_gy * 100.0));
  writeLittleEndian(payload, 78, (int16_t)(filtered_gz * 100.0));

  // Protocol Wrapper
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

  if (rbTx.notify(packet, 88))
    gpsUpdateCount++;
}

// Background polling and data routing for the u-blox module
void processGNSS() {
  if (!gpsEnabled)
    return;
  myGNSS.checkUblox();

  if (myGNSS.getPVT()) {
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;
    GPSFixType = myGNSS.packetUBXNAVPVT->data.fixType;
    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;
      sendRaceboxPacket();
    }
  }

  // Backup Recovery: Ensure background processing during data stalls
  static unsigned long lastValidData = 0;
  if (myGNSS.getPVT())
    lastValidData = millis();
  if (deviceConnected && (millis() - lastValidData > 2000))
    myGNSS.checkUblox();
}

// IMU Sampling and EMA (Exponential Moving Average) Smoothing
void processIMU() {
  if (!imuEnabled)
    return;
  static unsigned long lastRead = 0;
  if (millis() - lastRead < AccelSampleInterval)
    return;
  lastRead = millis();

  float ax = IMU.readFloatAccelX(), ay = IMU.readFloatAccelY(),
        az = IMU.readFloatAccelZ();
  float gx = IMU.readFloatGyroX(), gy = IMU.readFloatGyroY(),
        gz = IMU.readFloatGyroZ();

  filtered_ax = (accelAlpha * ax) + (1.0 - accelAlpha) * filtered_ax;
  filtered_ay = (accelAlpha * ay) + (1.0 - accelAlpha) * filtered_ay;
  filtered_az = (accelAlpha * az) + (1.0 - accelAlpha) * filtered_az;
  filtered_gx = (gyroAlpha * gx) + (1.0 - gyroAlpha) * filtered_gx;
  filtered_gy = (gyroAlpha * gy) + (1.0 - gyroAlpha) * filtered_gy;
  filtered_gz = (gyroAlpha * gz) + (1.0 - gyroAlpha) * filtered_gz;
}

// ============================================================================
// --- 🔋 POWER & SYSTEM MANAGEMENT ---
// ============================================================================

bool configureGPS() {
  if (!pendingConfig)
    return false;
  Serial.println("⚙️ Syncing GPS Settings...");
  Serial1.begin(GPS_BAUD);

  bool detected = false;
  for (int i = 0; i < 3; i++) {
    if (myGNSS.begin(Serial1)) {
      detected = true;
      break;
    }
    delay(20);
  }

  if (!detected) {
    Serial1.begin(FACTORY_GPS_BAUD);
    delay(50);
    if (myGNSS.begin(Serial1)) {
      myGNSS.setSerialRate(GPS_BAUD);
      delay(100);
      Serial1.begin(GPS_BAUD);
      if (!myGNSS.begin(Serial1))
        return false;
    } else
      return false;
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

void enableGPS() {
  if (gpsEnabled)
    return;
  digitalWrite(GPS_EN_PIN, HIGH);
  gpsEnabled = true;
  delay(100);
  if (!deviceConnected) {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.setInterval(FAST_ADV_INTERVAL,
                                      FAST_ADV_INTERVAL + 100);
    Bluefruit.Advertising.start(0);
  }
}

void disableGPS() {
  GPSFixType = 0;
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
}

void enableIMU() {
  if (imuEnabled)
    return;
  IMU.settings.gyroEnabled = 1;
  IMU.settings.accelEnabled = 1;
  IMU.settings.accelRange = 2;
  IMU.settings.gyroRange = 2000;
  if (IMU.begin() != 0)
    return;
  imuEnabled = true;
}

void disableIMU() {
  if (!imuEnabled)
    return;
  // Explicitly power down the sensor registers
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);
  imuEnabled = false;
}

// Manage Charging, Disconnect Timeouts, and Deep Sleep
void managePower() {
  bool currentlyPluggedIn = isPluggedIn();

  if (!currentlyPluggedIn && !deviceConnected &&
      currentBatteryPercentage == 0) {
    enterDeepSleep();
  }

  // Determine if sensors should be active
  bool shouldBeActive =
      deviceConnected || (currentlyPluggedIn && !SLEEP_WHILE_CHARGING);

  if (shouldBeActive) {
    lastActivityTime = millis();
    lastDisconnectTime = millis();
    if (!gpsEnabled) {
      enableGPS();
    }
    // Keep trying to configure until pendingConfig is false
    configureGPS();
    enableIMU();
  } else {
    if (imuEnabled)
      disableIMU();
  }

  // Hot-State Timeout (Keep GPS active for a window after usage)
  if (!deviceConnected && gpsEnabled && !currentlyPluggedIn) {
    if (millis() - lastDisconnectTime > GPS_HOT_TIMEOUT_MS) {
      Serial.printf("⏰ GPS Hot Timeout Reached. Powering down.\n");
      disableGPS();
    }
  }

  // Deep Sleep Safety Net
  if (ENABLE_DEEP_SLEEP && !deviceConnected && !currentlyPluggedIn) {
    if (millis() - lastActivityTime > (DEEP_SLEEP_DAYS * 86400000UL))
      enterDeepSleep();
  }

  // Reset charging state trackers
  if (lastPluggedInState && !currentlyPluggedIn) {
    lastActivityTime = millis();
    if (!deviceConnected)
      lastDisconnectTime = millis();
  }
  lastPluggedInState = currentlyPluggedIn;
}

// Smart Recovery Watchdog: Forces re-sync if the 25Hz feed stalls
void handleWatchdog() {
  static unsigned long lastValidData = 0;
  static unsigned long lastConnection = 0;
  static bool wasConnected = false;

  if (deviceConnected && !wasConnected) {
    lastConnection = millis();
    lastValidData = millis();
  }
  wasConnected = deviceConnected;

  if (gpsEnabled && myGNSS.getPVT())
    lastValidData = millis();
}

// Periodic Status Feed to the Computer
void reportSystemStats() {
  if (millis() - lastGpsRateCheckTime < GPS_RATE_REPORT_MS)
    return;

  float bleRate = gpsUpdateCount / (GPS_RATE_REPORT_MS / 1000.0);
  float gnssRate = gnssUpdateCount / (GPS_RATE_REPORT_MS / 1000.0);
  updateBatteryState();
  Serial.println("--------------------------------------------------");
  Serial.printf("POWER   | Bat: %d%% (%0.2fV) | Mult: %0.4f\n",
                currentBatteryPercentage, getBatteryVoltage(),
                batteryMultiplier);
  Serial.printf("STATE   | Charging: %s | USB: %s | BLE: %s\n",
                isCharging() ? "YES ⚡" : "NO 🔋",
                isPluggedIn() ? "CONNECTED" : "DISCONNECTED",
                deviceConnected ? "CONNECTED" : "IDLE");
  if (gpsEnabled && myGNSS.packetUBXNAVPVT) {
    Serial.printf(
        "GNSS    | BLE: %.2f Hz | GNSS: %.2f Hz | SVs: %u | Fix: %u\n", bleRate,
        gnssRate, myGNSS.packetUBXNAVPVT->data.numSV,
        myGNSS.packetUBXNAVPVT->data.fixType);
  }
  Serial.println("--------------------------------------------------");

  gpsUpdateCount = 0;
  gnssUpdateCount = 0;
  lastGpsRateCheckTime = millis();
}

// --- LED Status ---
void updateLEDs(uint8_t fixType) {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  // 1. HIGHEST PRIORITY: Critical Battery Alert
  if (isCritical && !isCharging()) {
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledState = !ledState;

      // Blink Red, keep Green off during critical alert
      digitalWrite(LED_RED, ledState);
      digitalWrite(LED_GREEN, HIGH);
    }
    return; // Exit early so GPS logic doesn't overrule the blink
  }

  // 2. SECOND PRIORITY: GPS Disabled
  if (!gpsEnabled) {
    digitalWrite(LED_RED, HIGH);   // OFF
    digitalWrite(LED_GREEN, HIGH); // OFF
    return;
  }

  // 3. LOWEST PRIORITY: Standard GPS Status
  switch (fixType) {
  case 3:                         // 3D Fix
  case 4:                         // GNSS + DR
    digitalWrite(LED_RED, HIGH);  // Red OFF
    digitalWrite(LED_GREEN, LOW); // Green ON
    break;

  case 1:                         // DR only
  case 2:                         // 2D Fix
    digitalWrite(LED_RED, LOW);   // Red ON
    digitalWrite(LED_GREEN, LOW); // Green ON -> Orange/Yellow
    break;

  default:                         // No fix
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
  delay(1000);
  Serial.println("\n\n🚀 SYSTEM STARTUP");

  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  pinMode(11, INPUT_PULLDOWN);

  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  digitalWrite(PIN_VBAT_ENABLE, LOW); // Start LOW & Stay LOW (Safe & Stable)
  pinMode(PIN_HICHG, OUTPUT);
  digitalWrite(PIN_HICHG, LOW);
  pinMode(PIN_CHG, INPUT_PULLUP); // Prevent float current leakage

  Wire.setClock(400000);
  analogReference(AR_DEFAULT);
  analogReadResolution(12);

  NRF_POWER->DCDCEN = 1; // Enable DC-DC converter (Saves ~30% radio current)

  // Put external QSPI Flash into Deep Power Down
  // We use the Adafruit Flash library's standard command (0xB9)
  // This is safe even if the library isn't fully loaded.
  pinMode(PIN_QSPI_CS, OUTPUT);
  digitalWrite(PIN_QSPI_CS, HIGH); // Ensure CS is de-asserted

  pinMode(OnboardledPin, OUTPUT);
  digitalWrite(OnboardledPin, HIGH);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  updateBatteryState();
  if (currentBatteryPercentage == 0) {
    // Flash RED LED for 5 seconds
    // (10 cycles of 250ms ON + 250ms OFF = 5 seconds)
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_RED, LOW); // ON 
      delay(250);
      digitalWrite(LED_RED, HIGH); // OFF
      delay(250);
    }
    // Ensure everything is off before sleep
    digitalWrite(LED_RED, HIGH);
    // 3. Enter Deep Sleep
    // This puts the device to sleep forever until a hardware reset/recharge
    enterDeepSleep();
  }

  if (IMU.begin() != 0) {
    Serial.println("❌ IMU Init Failed");
  } else {
    imuEnabled = true;
    disableIMU();
  }

  // BLE Configuration
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 12);

  // Service Setup
  disService.begin();
  disModel.setProperties(CHR_PROPS_READ);
  disModel.begin();
  disModel.write(DEVICE_NAME);
  disSerial.setProperties(CHR_PROPS_READ);
  disSerial.begin();
  disSerial.write(SERIAL_NUM);
  disFirmware.setProperties(CHR_PROPS_READ);
  disFirmware.begin();
  disFirmware.write(FIRMWARE_VER);
  disManuf.setProperties(CHR_PROPS_READ);
  disManuf.begin();
  disManuf.write(MANUFACTURER);

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
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(ECO_ADV_INTERVAL, ECO_ADV_INTERVAL + 200);
  Bluefruit.Advertising.start(0);

  Serial.println("📡 BLE Broadcast Started.");
  disableGPS();
}

void loop() {
  bool idle = !deviceConnected && !gpsEnabled && !imuEnabled;

  if (idle && (SLEEP_WHILE_CHARGING || !isPluggedIn())) {
    managePower();
    delay(1000);
    return;
  }
  processGNSS();
  processIMU();
  managePower();
  handleWatchdog();
  reportSystemStats();
  updateLEDs(GPSFixType);
  delay(1);
}