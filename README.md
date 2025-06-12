# Open Source RaceBox Mini Emulator

## Overview

This project provides firmware for an ESP32-based device that acts as a high-performance GPS/IMU  Bluetooth Low Energy (BLE) broadcaster. It integrates a U-blox GNSS module for precise position, velocity, and timing data at 25Hz, along with an MPU6050 accelerometer and gyroscope for motion sensing. All collected data is streamed over BLE, making it ideal for applications like vehicle performance analysis, lap timing, or real-time telemetry.

The device is designed to be housed in a custom 3D-printed enclosure, with design files included in this repository.

---

## Features

- **High-Resolution GPS Data**: Achieves 25Hz update rate using a U-blox GNSS module.  
- **Multi-Constellation Support**: Leverages GPS, Galileo, GLONASS, and BeiDou for enhanced accuracy and reliability.  
- **Integrated IMU**: Captures 3-axis accelerometer (`±8g`) and 3-axis gyroscope (`±500 deg/s`) data from the MPU6050.  
- **Real-time BLE Streaming**: Transmits a custom 88-byte data packet containing comprehensive GNSS and IMU information over BLE.  
- **Automatic Configuration**: Robust GPS initialization.
- **Custom RaceBox Protocol**: Data is encapsulated in a UBX-like custom protocol for efficient transmission.  
- **3D Printable Enclosure**: Includes design files for a compact and protective case.  

---

## Bill of Materials (BOM)

To build this project, you'll need the following components:

- **ESP32 Development Board**: (https://a.co/d/fTThpTl)  
- **U-blox GNSS Module**: (https://a.co/d/b6DcUS6 or https://a.co/d/54zrba3)
- **MPU6050 Accelerometer/Gyroscope Module**: (https://a.co/d/dCMwffg or similar)  
- **24AWG Hook-up Wire/Jumper Wires** or just use the wiring that comes with GNSS module, its perfect for this application.
- **M3x6 Screws** (2)    
- **Soldering Iron & Solder**
- **USB-C or Micro-USB Cable** (Depending on your ESP32 board, But USB-C all the things)  
- **3D Printer & ABS Filament**  

The electronics should cost under 40$. 

---

## Assembly Instructions

### Wire the Components, load the firmware and test them ourdoors.

**ESP32 <--> U-blox GNSS Module:**
- ESP32 GPIO 16 (RX2) <--> GNSS TX  
- ESP32 GPIO 17 (TX2) <--> GNSS RX  
- ESP32 5V or 3.3V <--> GNSS VCC *(check voltage requirements)*  
- ESP32 GND <--> GNSS GND  

**ESP32 <--> MPU6050:**
- ESP32 GPIO 21 (SDA) <--> MPU6050 SDA  
- ESP32 GPIO 22 (SCL) <--> MPU6050 SCL  
- ESP32 5V or 3.3V <--> MPU6050 VCC *(check voltage requirements)*
- ESP32 GND <--> MPU6050 GND  


---

### Prepare the 3D Printed Case

- You shouldn't need any post processing of the printed parts but make sure the buttons slide smoothly in the holes in the lid

![image info](Images/Component%20Arrangement.jpg "Component arrangement")

### Enclose the Electronics

- Mount the MPU6050 module using M3 screws
![image info](Images/Accelerometer_mount.jpg "Wiring Guide")
- Place the ESP32 in the slot  
- Route cables neatly
![image info](Images/Wiring%20Guide.jpg "Wiring Guide")
- Close the case carefully, it should click together. Make sure none of the wires are pinced or in the way of the buttons
![alt text](Images/GoPro%20base%20Assembled.jpg "GoPro mount version")
![alt text](Images/Base%20Assembled.jpg "Basic version")

---

## Firmware Setup and Upload

### 1. Install Arduino IDE

Download from the [official Arduino website](https://www.arduino.cc/en/software).

### 2. Install ESP32 Board Support

- Open Arduino IDE Preferences  
- In “Additional Board Manager URLs” add: 
    - https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

### 3. Install Libraries:
- Go to Sketch > Include Library > Manage Libraries... and install the following:
    - Adafruit MPU6050
    - Adafruit Unified Sensor
    - SparkFun u-blox GNSS Arduino Library (ensure it's version 2)
    - BLEDevice (This is usually part of the ESP32 board package, but confirm it's available)
### 4. Open the Sketch: 
- Open the esp32_racebox_mini_emulator.ino in the Arduino IDE.
    - Configure Board Settings:
    - Go to Tools > Board and select your specific ESP32 board (e.g., "ESP32 Dev Module").
    - Ensure the correct Upload Speed and Port are selected.
### 5. Upload the Firmware:
- *OPTIONAL*- Update deviceName(line 33) to whatever you want (the format is "RaceBox Mini {10 digit number}").
- Click the "Upload" button (right arrow icon) in the Arduino IDE to compile and upload the code to your ESP32.
    - Note: Sometimes during the upload, I had to hold the BOOT button, press and release the EN button then release the BOOT button for the upload to work.
- Check the Serial Monitor and ensure everything starts up succesfully, initialization is complete when it starts displaying "BLE Packet Rate: 0.00 Hz" every 5 seconds.

## 3D Printed Enclosure

The repository includes `*.3mf` files for the custom 3D-printed enclosure designed specifically for the components in the BOM.

* 1x Base file.(either the GoPro mount version or the basic one to use with velcro)
* 1x Lid depending on the the type of GPS module you have.
* 2x Buttons
* 1x axdl_mount

Theres a Accelrometer blank included if you have a different accelrometer to create your own mount.

### Recommended Print Settings:

* **Material:** ABS (for durability and heat resistance, especially if used in a car)
* **Layer Height:** 0.2mm
* **Infill:** 15-20% (for sufficient strength)
* **Nozzle Temperature:** (Refer to your filament manufacturer's recommendations)
* **Bed Temperature:** (Refer to your filament manufacturer's recommendations)

## Usage

Once the firmware is uploaded and the device is powered on:

1.  The ESP32 will start advertising a BLE service named "**RaceBox Mini 0123456789**"(the devicename you customized before uploading the firmware).
2.  Use a compatible BLE client application to connect to the device.
    - Tested compatible apps, theoreticaly anything that supports a racebox mini should work (except the RaceBox app).
        - Solostorm (Android)
        - NMEAconnect (iOS)
        - AutoX DL(iOS)

## Contribution

Feel free to open issues or submit pull requests if you have suggestions, improvements, or bug fixes.

## License

This project is open-source and available under the [MIT License](LICENSE.md).