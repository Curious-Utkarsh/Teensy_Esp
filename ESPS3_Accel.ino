#include "IIS3DWB.h"
#include "SPI.h"
#include <Arduino.h>

#define SerialDebug true  // set to true to get Serial output for debugging

// --- PIN DEFINITIONS ---
// Define SPI pins
const int csPin = 1;     // Chip Select Pin for IIS3DWB
const int mosiPin = 9;   // "MOSI" Pin (D9 on XIAO ESP32-S3)
const int misoPin = 8;   // "MISO" Pin (D8 on XIAO ESP32-S3)
const int sckPin = 7;    // SCK Pin (D7 on XIAO ESP32-S3)

// --- SENSOR & STATE VARIABLES ---
String control_val = "";

// IIS3DWB sensor settings
uint8_t Ascale = AFS_2G; // Accelerometer Full Scale (AFS_2G, AFS_4G, AFS_8G, AFS_16G)
float aRes;              // Scale resolution per LSB for the accelerometer
float accelBias[3] = {0.0f, 0.0f, 0.0f}; // Offset biases for the accelerometer
int16_t IIS3DWBData[3] = {0};             // Stores the 16-bit signed sensor output
int16_t ax, ay, az;                       // Variables to hold latest accel data values in milli-g's
const float acc_mult_factor = 1000.0;     // Conversion factor to get milli-g's

// Flags and counters
bool Send_Garbage_Flag = false;
int16_t Garbage_value = -5000;
int count = 10;

// Instantiate IIS3DWB class with the Chip Select pin
IIS3DWB IIS3DWB(csPin); 

// --- FUNCTION DEFINITIONS ---

// Function to set up the IIS3DWB sensor
void IIS3DWB_SETUP() {
  if (SerialDebug) {
    //Serial.println("Setting up IIS3DWB...");
  }

  // Reset IIS3DWB to start fresh
  IIS3DWB.reset();
  delay(100); // Allow time for reset

  // Get accel sensor resolution, only need to do this once
  aRes = IIS3DWB.getAres(Ascale);

  // Initialize the sensor with the chosen scale
  IIS3DWB.init(Ascale);

  // Apply any pre-calculated offset biases
  IIS3DWB.offsetBias(accelBias);
  delay(10);

  if (SerialDebug) {
    //Serial.println("IIS3DWB setup complete.");
  }
}

// Function to check communication with the sensor by reading its Chip ID
uint8_t IIS3DWB_CHECK() {
  int initial_count = 10;
  while ((IIS3DWB.getChipID() != 0x7B) && (initial_count > 0)) {
    delayMicroseconds(10);
    initial_count--;
  }
  return IIS3DWB.getChipID();
}


void setup() {
  Serial.begin(1500000);
  Serial1.begin(1500000, SERIAL_8N1, 43, 44);
  delay(2000); // Allow serial to initialize

  if (SerialDebug) {
    //Serial.println("Starting setup...");
  }

  // --- CONFIGURE PINS ---
  // Configure SPI Chip Select pin
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH); // Deselect device

  // --- INITIALIZE SPI ---
  // For ESP32, you must define the pins inside the begin() function
  // The parameters are: SCK, MISO, MOSI, CS (CS is optional, we handle it manually)
  SPI.begin(sckPin, misoPin, mosiPin);

  // --- SENSOR INITIALIZATION ---
  if (SerialDebug) {
    //Serial.println("Checking IIS3DWB communication...");
  }
  
  uint8_t c = IIS3DWB_CHECK();
  
  if (SerialDebug) {
    //Serial.print("IIS3DWB Chip ID: 0x");
    //Serial.print(c, HEX);
    //Serial.print(" (Expected: 0x7B)");
    //Serial.println();
  }

  // Check if communication was successful
  if (c == 0x7B) {
    IIS3DWB_SETUP();
    Send_Garbage_Flag = false;
  } else {
    if (SerialDebug) {
      //Serial.println("Error: IIS3DWB not found!");
    }
    Send_Garbage_Flag = true;
    // The loop will now send garbage data
  }

  if (SerialDebug) {
    //Serial.println("Setup complete. Starting loop.");
  }
}
/* End of setup */


void loop() {
  // The main loop already runs forever, so an inner for(;;) or while(1) is not needed.

    if (IIS3DWB.DRstatus() & 0x01) 
    {
      IIS3DWB.readAccelData(IIS3DWBData);

      // Calculate acceleration in milli-g's
      ax = acc_mult_factor * ((float)IIS3DWBData[0] * aRes - accelBias[0]);
      ay = acc_mult_factor * ((float)IIS3DWBData[1] * aRes - accelBias[1]);
      az = acc_mult_factor * ((float)IIS3DWBData[2] * aRes - accelBias[2]);

      // Send data over serial in binary format
      Serial1.write(0xAA); // Sync byte
      Serial1.write((uint8_t *)&(ax), sizeof(ax));
      Serial1.write((uint8_t *)&(ay), sizeof(ay));
      Serial1.write((uint8_t *)&(az), sizeof(az));
    }
    else 
   {
    // --- ERROR STATE ---
    // Send garbage data if the sensor is not detected
    Serial1.write(0xAA); // Sync byte
    Serial1.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));
    Serial1.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));
    Serial1.write((uint8_t *)&(Garbage_value), sizeof(Garbage_value));

    // Attempt to reconnect to the sensor

}
}

