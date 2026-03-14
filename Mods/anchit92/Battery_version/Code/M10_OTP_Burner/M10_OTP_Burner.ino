#include <Adafruit_TinyUSB.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>

SFE_UBLOX_GNSS myGNSS;

#define GPS_EN_PIN D1
#define PCB_VERSION

// --- U-BLOX M10 OTP PAYLOADS ---
const uint8_t M10_OTP_192MHZ_CLOCK[] = {
    0xB5, 0x62, 0x06, 0x41, 0x10, 0x00, 0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E,
    0x79, 0xBF, 0x28, 0xEF, 0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF, 0x8F, 0x0D,
    0xB5, 0x62, 0x06, 0x41, 0x1C, 0x00, 0x04, 0x01, 0xA4, 0x10, 0xBD, 0x34,
    0xF9, 0x12, 0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0,
    0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0xDE, 0xAE};

const uint8_t M10_OTP_115200_BAUD[] = {
    0xB5, 0x62, 0x06, 0x41, 0x14, 0x00, 0x04, 0x01, 0xA4, 0x08,
    0xDB, 0x36, 0x5D, 0x5D, 0x28, 0xEF, 0x12, 0x05, 0x01, 0x00,
    0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0x5B, 0xBD};

const uint8_t CMD_UBX_CFG_RST[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                   0x00, 0x00, 0x02, 0x00, 0x10, 0x68};
const uint8_t CMD_UBX_CFG_VERIFY[] = {0xB5, 0x62, 0x06, 0x8B, 0x14, 0x00, 0x00,
                                      0x04, 0x00, 0x00, 0x01, 0x00, 0xA4, 0x40,
                                      0x03, 0x00, 0xA4, 0x40, 0x05, 0x00, 0xA4,
                                      0x40, 0x0A, 0x00, 0xA4, 0x40, 0x4C, 0x15};
const uint8_t CMD_UBX_MON_VER[] = {0xB5, 0x62, 0x0A, 0x04,
                                   0x00, 0x00, 0x0E, 0x34};

// Expected VALGET response if OTP 192MHz is active
const uint8_t EXPECTED_OTP_VAL[] = {
    0xB5, 0x62, 0x06, 0x8B, 0x24, 0x00, 0x01, 0x04, 0x00, 0x00, 0x01,
    0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B, 0x03, 0x00, 0xA4, 0x40,
    0x00, 0xB0, 0x71, 0x0B, 0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71,
    0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0x76, 0x81};

void sendPayload(const uint8_t *payload, size_t len) {
  Serial1.write(payload, len);
  Serial1.flush();
  delay(500);
}

bool verifyOTP() {
  while (Serial1.available())
    Serial1.read();
  sendPayload(CMD_UBX_CFG_VERIFY, sizeof(CMD_UBX_CFG_VERIFY));

  uint8_t buf[100];
  int idx = 0;
  unsigned long start = millis();

  while (millis() - start < 1500 && idx < sizeof(EXPECTED_OTP_VAL)) {
    if (Serial1.available()) {
      uint8_t c = Serial1.read();
      if (idx == 0) {
        if (c == 0xB5)
          buf[idx++] = c;
      } else if (idx == 1) {
        if (c == 0x62)
          buf[idx++] = c;
        else
          idx = 0; // Reset if not UBX header
      } else if (idx == 2) {
        if (c == 0x06)
          buf[idx++] = c;
        else
          idx = 0; // We strictly want Class 0x06
      } else if (idx == 3) {
        if (c == 0x8B)
          buf[idx++] = c;
        else
          idx = 0; // We strictly want ID 0x8B
      } else {
        buf[idx++] = c; // It is our exact payload, digest the rest!
      }
    }
  }
  return (idx == sizeof(EXPECTED_OTP_VAL) &&
          memcmp(buf, EXPECTED_OTP_VAL, idx) == 0);
}

long detectBaudRate() {
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(500);
  digitalWrite(GPS_EN_PIN, LOW);
#else
  digitalWrite(GPS_EN_PIN, LOW);
  delay(500);
  digitalWrite(GPS_EN_PIN, HIGH);
#endif
  delay(1500);

  long bauds[] = {115200, 9600, 38400, 19200};
  for (int i = 0; i < 4; i++) {
    Serial1.begin(bauds[i]);
    delay(100);
    while (Serial1.available())
      Serial1.read();
    sendPayload(CMD_UBX_MON_VER, sizeof(CMD_UBX_MON_VER));

    unsigned long start = millis();
    bool b5 = false;
    while (millis() - start < 500) {
      if (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (b == 0xB5)
          b5 = true;
        else if (b5 && b == 0x62)
          return bauds[i];
        else
          b5 = false;
      }
    }
    Serial1.end();
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  pinMode(GPS_EN_PIN, OUTPUT);

  Serial.println("\n--- M10 OTP Check & Burn ---");

  long currentBaud = detectBaudRate();
  if (currentBaud == 0) {
    Serial.println("❌ GNSS not found.");
    return;
  }

  Serial1.begin(currentBaud);
  bool otpDone = verifyOTP();

  Serial.printf("Current Baud: %ld\n", currentBaud);
  Serial.printf("OTP 192MHz: %s\n", otpDone ? "VERIFIED" : "NOT SET");

  if (currentBaud == 115200 && otpDone) {
    Serial.println("✅ Module is fully configured. No action needed.");
  } else {
    Serial.println("⚠️ Configuration mismatch. Starting burn sequence...");
    delay(2000);

    if (currentBaud != 115200) {
      Serial.println("-> Burning 115200 Baud OTP...");
      sendPayload(M10_OTP_115200_BAUD, sizeof(M10_OTP_115200_BAUD));
    }

    if (!otpDone) {
      Serial.println("-> Burning 192MHz Clock OTP...");
      sendPayload(M10_OTP_192MHZ_CLOCK, sizeof(M10_OTP_192MHZ_CLOCK));
    }

    Serial.println("-> Resetting Module...");
    sendPayload(CMD_UBX_CFG_RST, sizeof(CMD_UBX_CFG_RST));
    delay(3000);

    Serial1.end();
    Serial1.begin(115200);

    Serial.println("-> Final Validation...");
    if (verifyOTP()) {
      Serial.println("✅ Burn Successful! 115200 Baud & 192MHz Active.");
      myGNSS.begin(Serial1);
      myGNSS.saveConfiguration();
    } else {
      Serial.println("❌ Burn Failed or partial write. Check wiring/power.");
    }
  }
  Serial.println("--- System Halted ---");
}

void loop() { delay(1000); }