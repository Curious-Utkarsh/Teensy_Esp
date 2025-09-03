/**
 * ESP32 + MPU9250 (HSPI) + Serial0 Telemetry
 * 
 * Connections (HSPI default pins):
 * - CS   -> GPIO 15
 * - MOSI -> GPIO 13
 * - MISO -> GPIO 12
 * - SCK  -> GPIO 14
 * 
 * Debug / Telemetry Output:
 * - USB Serial0 (Serial Monitor)
 */

#include <Arduino.h>
#include <SPI.h>
#include <MPU9250_WE.h>

// --- HSPI Configuration ---
#define HSPI_CS_PIN   15
#define HSPI_MOSI_PIN 13
#define HSPI_MISO_PIN 12
#define HSPI_SCK_PIN  14

// Create HSPI bus instance
SPIClass hspi(HSPI);

// Instantiate MPU9250 on HSPI
MPU9250_WE myMPU9250 = MPU9250_WE(&hspi, HSPI_CS_PIN, HSPI_MOSI_PIN, HSPI_MISO_PIN, HSPI_SCK_PIN, true);

// Data sync byte
const int8_t SYNC_BYTE = 0xAA;

// Raw data variables
int16_t gx, gy, gz;

void setup() {
  // Start USB Serial for telemetry
  Serial.begin(1500000);
  delay(2000);
  Serial.println("=== MPU9250 HSPI Telemetry ===");

  // Init HSPI
  hspi.begin(HSPI_SCK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN, HSPI_CS_PIN);
  pinMode(HSPI_CS_PIN, OUTPUT);

  // Init MPU9250
  if (!myMPU9250.init()) {
    Serial.println("MPU9250 not found! Check wiring.");
    while (1);
  }

  Serial.println("MPU9250 initialized. Calibrating...");

  myMPU9250.autoOffsets();  // gyro/accel offsets
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  Serial.println("Calibration done. Streaming gyro raw values...");
}

void loop() {
  // Read raw gyro values
  xyzFloat gyr = myMPU9250.getGyrRawValues();
  gx = gyr.x;
  gy = gyr.y;
  gz = gyr.z;

  // Send in binary format: [SYNC][gx][gy][gz]
  Serial.write(SYNC_BYTE);
  Serial.write((uint8_t *)&gx, sizeof(gx));
  Serial.write((uint8_t *)&gy, sizeof(gy));
  Serial.write((uint8_t *)&gz, sizeof(gz));

  delay(1); // Prevent hogging CPU
}
