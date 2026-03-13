#include <Adafruit_TinyUSB.h>

// ==========================================
// CONFIGURATION - Change these and re-flash
// ==========================================
#define GPS_BAUD 9600 // Try 9600, 38400, or 115200
#define GPS_EN_PIN D1 // Power enable pin
// ==========================================

// Uncomment the line below if using the custom PCB version where
// the GPS_EN_PIN logic is inverted. (For PCB: LOW = ON, HIGH = OFF)
// #define PCB_VERSION

void setup() {
  // USB Serial for computer
  Serial.begin(115200);

  // Power on the GPS module
  pinMode(GPS_EN_PIN, OUTPUT);
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, LOW);
#else
  digitalWrite(GPS_EN_PIN, HIGH);
#endif

  // GNSS Serial (Hardcoded pins for XIAO D6/D7)
  Serial1.begin(GPS_BAUD);

  // Wait for Serial Monitor to open (optional)
  delay(2000);
  Serial.print("--- Minimal GNSS Passthrough @ ");
  Serial.print(GPS_BAUD);
  Serial.println(" baud ---");
}

void loop() {
  // Read from GPS, send to Computer
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  // Read from Computer, send to GPS (for sending commands)
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
}
