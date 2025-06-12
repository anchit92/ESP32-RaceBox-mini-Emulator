#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- GPS Configuration ---
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 115200
#define FACTORY_GPS_BAUD 9600

SFE_UBLOX_GNSS myGNSS;
HardwareSerial GPS_Serial(2);

Adafruit_MPU6050 mpu;

// --- BLE Configuration ---
const char* const RACEBOX_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const RACEBOX_CHARACTERISTIC_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const RACEBOX_CHARACTERISTIC_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicTx = NULL;
BLECharacteristic *pCharacteristicRx = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

const String deviceName = "RaceBox Mini 0123456789";

// --- Packet Timing ---
unsigned long lastPacketSendTime = 0;
const unsigned long PACKET_SEND_INTERVAL_MS = 40;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
const unsigned long GPS_RATE_REPORT_INTERVAL_MS = 5000;

// --- BLE Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("✅ BLE Client connected");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("❌ BLE Client disconnected");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("📨 Received BLE command: ");
      for (char c : rxValue)
        Serial.printf("0x%02X ", (uint8_t)c);
      Serial.println();
    }
  }
};

// --- UBX Packet Construction Helpers ---
template <typename T>
void writeLittleEndian(uint8_t* buffer, int offset, T value) {
  memcpy(buffer + offset, &value, sizeof(T));
}

void calculateChecksum(uint8_t* payload, uint16_t len, uint8_t cls, uint8_t id, uint8_t* ckA, uint8_t* ckB) {
  *ckA = *ckB = 0;
  *ckA += cls; *ckB += *ckA;
  *ckA += id; *ckB += *ckA;
  *ckA += len & 0xFF; *ckB += *ckA;
  *ckA += len >> 8; *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

void resetGpsBaudRate() {
  Serial.println("Attempting to set Correct Baud Rate");
  GPS_Serial.begin(FACTORY_GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("u-blox GNSS not detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while(1) delay(100);
  } else {
    Serial.print("GNSS detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud!");
  }
  delay(500);

  // Now switch baud rate
  Serial.print("Setting baud rate to ");
  Serial.print(GPS_BAUD);
  Serial.println("...");
  myGNSS.setSerialRate(GPS_BAUD);
  Serial.print("Baud rate changed to ");
  Serial.println(GPS_BAUD);

  GPS_Serial.end();
  delay(100);
  // Re-initialize the serial port at the new baud rate
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("GNSS not detected at ");
    Serial.print(GPS_BAUD);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while (1) delay(100);
  }
  Serial.print("GNSS detected at ");
  Serial.print(GPS_BAUD);
  Serial.println(" baud! Saving to Flash");
  myGNSS.saveConfiguration(); // Save to flash
  GPS_Serial.end();
}

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip");
    while (1) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  if (!myGNSS.begin(GPS_Serial)) {
    Serial.println("❌ GNSS not detected. Attempting to configure.");
    GPS_Serial.end();
    resetGpsBaudRate();
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  }

  // Set GNSS output to PVT only
  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    // --- Configure GPS update rate to 25 Hz ---
  if (myGNSS.setNavigationFrequency(25)) {
    Serial.println("✅ GPS update rate set to 25 Hz.");
  } else {
    Serial.println("❌ Failed to set GPS update rate.");
  }

  // --- Enable multiple GNSS constellations ---
  // The specific constellations available depend on your u-blox module.
  // Common ones are GPS, Galileo, GLONASS, BeiDou, QZSS, SBAS.

  // Enable GPS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS)) {
    Serial.println("✅ GPS enabled.");
  } else {
    Serial.println("❌ Failed to enable GPS.");
  }

  // Enable Galileo
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO)) {
    Serial.println("✅ Galileo enabled.");
  } else {
    Serial.println("❌ Failed to enable Galileo.");
  }

  // Enable GLONASS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS)) {
    Serial.println("✅ GLONASS enabled.");
  } else {
    Serial.println("❌ Failed to enable GLONASS.");
  }
  
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU)) {
    Serial.println("✅ BEIDOU enabled.");
  } else {
    Serial.println("❌ Failed to enable BEIDOU.");
  }

  // --- BLE Setup ---
  BLEDevice::init(deviceName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(RACEBOX_SERVICE_UUID);
  pCharacteristicTx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTx->addDescriptor(new BLE2902());
  pCharacteristicRx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristicRx->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(RACEBOX_SERVICE_UUID);
  BLEDevice::startAdvertising();
  Serial.println("📡 BLE advertising started.");

  lastGpsRateCheckTime = millis();
}

void loop() {
  myGNSS.checkUblox(); // Required to keep GNSS data flowing
  
  if (deviceConnected && myGNSS.getPVT() && myGNSS.packetUBXNAVPVT != NULL) {
    const unsigned long now = millis();
    lastPacketSendTime = now;
    gpsUpdateCount++;
    // Now that we're sending a packet, read the acceloromter
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Convert accelerometer to milli-g (1g = 9.80665 m/s^2)
    int16_t gX = a.acceleration.x * 1000.0 / 9.80665;
    int16_t gY = a.acceleration.y * 1000.0 / 9.80665;
    int16_t gZ = a.acceleration.z * 1000.0 / 9.80665;

    // Convert gyro to centi-deg/sec
    int16_t rX = g.gyro.x * 180.0 / M_PI * 100.0;
    int16_t rY = g.gyro.y * 180.0 / M_PI * 100.0;
    int16_t rZ = g.gyro.z * 180.0 / M_PI * 100.0;
    uint8_t payload[80] = {0};
    uint8_t packet[88] = {0};

    // Access data directly from myGNSS.packetUBXNAVPVT->data
    writeLittleEndian(payload, 0, myGNSS.packetUBXNAVPVT->data.iTOW);
    writeLittleEndian(payload, 4, myGNSS.packetUBXNAVPVT->data.year);
    writeLittleEndian(payload, 6, myGNSS.packetUBXNAVPVT->data.month);
    writeLittleEndian(payload, 7, myGNSS.packetUBXNAVPVT->data.day);
    writeLittleEndian(payload, 8, myGNSS.packetUBXNAVPVT->data.hour);
    writeLittleEndian(payload, 9, myGNSS.packetUBXNAVPVT->data.min);
    writeLittleEndian(payload, 10, myGNSS.packetUBXNAVPVT->data.sec);

    // Offset 11: Validity Flags (RaceBox Protocol) 
    uint8_t raceboxValidityFlags = 0;
    if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate) raceboxValidityFlags |= (1 << 0); // Bit 0: valid date 
    if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime) raceboxValidityFlags |= (1 << 1); // Bit 1: valid time 
    if (myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxValidityFlags |= (1 << 2); // Bit 2: fully resolved 
    writeLittleEndian(payload, 11, raceboxValidityFlags);

    // Offset 12: Time Accuracy (RaceBox Protocol) 
    writeLittleEndian(payload, 12, myGNSS.packetUBXNAVPVT->data.tAcc);

    // Offset 16: Nanoseconds (RaceBox Protocol) 
    writeLittleEndian(payload, 16, myGNSS.packetUBXNAVPVT->data.nano);

    // Offset 20: Fix Status (RaceBox Protocol) 
    writeLittleEndian(payload, 20, myGNSS.packetUBXNAVPVT->data.fixType);

    // Offset 21: Fix Status Flags (RaceBox Protocol)
    uint8_t fixStatusFlagsRacebox = 0;

    if (myGNSS.packetUBXNAVPVT->data.fixType == 3) {
        fixStatusFlagsRacebox |= (1 << 0); // Bit 0: valid fix 
    }
    writeLittleEndian(payload, 21, fixStatusFlagsRacebox);

    // Offset 22: Date/Time Flags (RaceBox Protocol) 
    uint8_t raceboxDateTimeFlags = 0;
    if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime) raceboxDateTimeFlags |= (1 << 5); // Available confirmation of Date/Time Validity
    if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate) raceboxDateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
    if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime && myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxDateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
    writeLittleEndian(payload, 22, raceboxDateTimeFlags);

    // Offset 23: Number of SVs (RaceBox Protocol) 
    writeLittleEndian(payload, 23, myGNSS.packetUBXNAVPVT->data.numSV);

    // Remaining fields, mostly direct mappings from u-blox data
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

    // Offset 66: Lat/Lon Flags (RaceBox Protocol) 
    uint8_t latLonFlags = 0;
    if (myGNSS.packetUBXNAVPVT->data.fixType < 2) { // If no 2D/3D fix, then coordinates are considered invalid 
        latLonFlags |= (1 << 0); // Bit 0: Invalid Latitude, Longitude, WGS Altitude, and MSL Altitude
    }
    writeLittleEndian(payload, 66, latLonFlags);

    writeLittleEndian(payload, 68, gX);
    writeLittleEndian(payload, 70, gY);
    writeLittleEndian(payload, 72, gZ);
    writeLittleEndian(payload, 74, rX);
    writeLittleEndian(payload, 76, rY);
    writeLittleEndian(payload, 78, rZ);


    // Wrap in UBX (standard RaceBox header and checksum)
    packet[0] = 0xB5;
    packet[1] = 0x62;
    packet[2] = 0xFF; // Message Class: RaceBox Data Message 
    packet[3] = 0x01; // Message ID: RaceBox Data Message 
    packet[4] = 80;   // Payload size 
    packet[5] = 0;
    memcpy(packet + 6, payload, 80);
    uint8_t ckA, ckB;
    // Use the correct Class (0xFF) and ID (0x01) for checksum calculation as per RaceBox protocol 
    calculateChecksum(payload, 80, 0xFF, 0x01, &ckA, &ckB);
    packet[86] = ckA;
    packet[87] = ckB;

    pCharacteristicTx->setValue(packet, 88);
    pCharacteristicTx->notify();
    delay(15);
  }

  // Report packet send rate
  const unsigned long now = millis();
  if ((now - lastGpsRateCheckTime) >= GPS_RATE_REPORT_INTERVAL_MS) {
    float rate = gpsUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
    Serial.printf("BLE Packet Rate: %.2f Hz\n", rate);
    gpsUpdateCount = 0;
    lastGpsRateCheckTime = now;
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}