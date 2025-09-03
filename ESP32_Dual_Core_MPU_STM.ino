/**
 * Core 0 Pipeline (IIS3DWB -> VSPI -> UART1):
 * - IIS3DWB CS    -> GPIO 5
 * - IIS3DWB MOSI  -> GPIO 23 (VSPI default)
 * - IIS3DWB MISO  -> GPIO 19 (VSPI default)
 * - IIS3DWB SCK   -> GPIO 18 (VSPI default)
 * - UART1 TX      -> GPIO 17 (Connect to your receiving device)
 *
 * Core 1 Pipeline (MPU9250 -> HSPI -> UART2):
 * - MPU9250 CS    -> GPIO 15
 * - MPU9250 MOSI  -> GPIO 13 (HSPI default)
 * - MPU9250 MISO  -> GPIO 12 (HSPI default)
 * - MPU9250 SCK   -> GPIO 14 (HSPI default)
 */

// --- LIBRARIES ---
#include <Arduino.h>
#include <SPI.h>
#include "IIS3DWB.h"      // Library for the IIS3DWB sensor
#include <MPU9250_WE.h>   // Library for the MPU9250 sensor
#include "esp_task_wdt.h" // For watchdog timer control

// --- GLOBAL CONFIGURATION ---
#define SERIAL_DEBUG true // Set to true to get diagnostic output on the USB Serial Monitor

// --- CORE 0: IIS3DWB SENSOR (ON VSPI BUS) ---
#define VSPI_CS_PIN 5
#define VSPI_MOSI_PIN 23
#define VSPI_MISO_PIN 19
#define VSPI_SCK_PIN 18
#define UART1_TX_PIN 17

// Instantiate IIS3DWB class. This library uses the default global SPI instance.
// We will ensure the global SPI is configured for VSPI within Core 0's task.
IIS3DWB iis3dwb(VSPI_CS_PIN);

// IIS3DWB sensor settings
const uint8_t Ascale = AFS_2G; // Accelerometer Full Scale (AFS_2G, AFS_4G, AFS_8G, AFS_16G)
float aRes;                    // Scale resolution per LSB
float accelBias[3] = {0.0f, 0.0f, 0.0f}; // Offset biases
int16_t iis3dwbData[3] = {0};            // Raw sensor data
const float ACC_MULT_FACTOR = 1000.0;    // Conversion to milli-g's
int16_t garbageValue = -5000;

// --- CORE 1: MPU9250 SENSOR (ON HSPI BUS) ---
#define HSPI_CS_PIN 15
#define HSPI_MOSI_PIN 13
#define HSPI_MISO_PIN 12
#define HSPI_SCK_PIN 14

// Create a dedicated SPIClass instance for the HSPI bus
SPIClass hspi(HSPI);

// Instantiate MPU9250 class, passing the HSPI instance to the constructor
MPU9250_WE myMPU9250 = MPU9250_WE(&hspi, HSPI_CS_PIN, HSPI_MOSI_PIN, HSPI_MISO_PIN, HSPI_SCK_PIN, true);

// MPU9250 data variables
const int8_t SYNC_BYTE = 0xAA;
int16_t gx, gy, gz;

// --- TASK HANDLES ---
TaskHandle_t taskIIS_Core0;
TaskHandle_t taskMPU_Core1;


//================================================================================
//                            TASK FOR CORE 0: IIS3DWB
//================================================================================
void taskIIS(void *pvParameters) {
  if (SERIAL_DEBUG) {
    Serial.print("Task for IIS3DWB is running on core ");
    Serial.println(xPortGetCoreID());
  }

  // --- INITIALIZE HARDWARE FOR THIS CORE ---
  // Initialize the default SPI bus (VSPI) with its specific pins.
  // The IIS3DWB library uses this global SPI object implicitly.
  SPI.begin(VSPI_SCK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN);
  pinMode(VSPI_CS_PIN, OUTPUT);
  digitalWrite(VSPI_CS_PIN, HIGH);

  // --- SENSOR SETUP ---
  iis3dwb.reset();
  delay(100);

  // Check communication
  uint8_t chipID = iis3dwb.getChipID();
  if (chipID == 0x7B) {
    if (SERIAL_DEBUG) Serial.println("Core 0: IIS3DWB connection successful.");
    aRes = iis3dwb.getAres(Ascale);
    iis3dwb.init(Ascale);
    iis3dwb.offsetBias(accelBias);
  } else {
    if (SERIAL_DEBUG) {
      Serial.print("Core 0: Error, IIS3DWB not found! Chip ID: 0x");
      Serial.println(chipID, HEX);
    }
  }

  // --- TASK LOOP ---
  for (;;) {
    // Check if the sensor reported a successful connection
    if (chipID == 0x7B) {
      // Check if new data is available
      if (iis3dwb.DRstatus() & 0x01) {
        iis3dwb.readAccelData(iis3dwbData);

        // Calculate acceleration in milli-g's
        int16_t ax = ACC_MULT_FACTOR * ((float)iis3dwbData[0] * aRes - accelBias[0]);
        int16_t ay = ACC_MULT_FACTOR * ((float)iis3dwbData[1] * aRes - accelBias[1]);
        int16_t az = ACC_MULT_FACTOR * ((float)iis3dwbData[2] * aRes - accelBias[2]);

        // Send data over UART1 in binary format
        Serial1.write(SYNC_BYTE);
        
        Serial1.write((uint8_t *)&ax, sizeof(ax));
        Serial1.write((uint8_t *)&ay, sizeof(ay));
        Serial1.write((uint8_t *)&az, sizeof(az));
      }
    } else {
      // Send garbage data if the sensor was never detected
      Serial1.write(SYNC_BYTE);
      Serial1.write((uint8_t *)&garbageValue, sizeof(garbageValue));
      Serial1.write((uint8_t *)&garbageValue, sizeof(garbageValue));
      Serial1.write((uint8_t *)&garbageValue, sizeof(garbageValue));
    }
    
    // Give the scheduler time to run other tasks.
    // The IIS3DWB has a max ODR of 26.7kHz. A small delay is fine.
    vTaskDelay(1); 
    // taskYIELD();

  }
}

//================================================================================
//                            TASK FOR CORE 1: MPU9250
//================================================================================
void taskMPU(void *pvParameters) {
  if (SERIAL_DEBUG) {
    Serial.print("Task for MPU9250 is running on core ");
    Serial.println(xPortGetCoreID());
  }
  
  // --- INITIALIZE HARDWARE FOR THIS CORE ---
  // The HSPI pins and CS are already configured in the MPU9250_WE constructor.
  // We just need to call hspi.begin() here.
  hspi.begin(HSPI_SCK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN, HSPI_CS_PIN);
  pinMode(HSPI_CS_PIN, OUTPUT);
  
  // --- SENSOR SETUP ---
  // Disable watchdog timer as auto-offset calibration can be slow
  esp_task_wdt_delete(NULL); 
  
  if (SERIAL_DEBUG) Serial.println("Core 1: Initializing MPU9250...");
  
  myMPU9250.init();
  delay(2000);
  if (SERIAL_DEBUG) Serial.println("Core 1: Calibrating MPU9250 offsets...");
  myMPU9250.autoOffsets();
  if (SERIAL_DEBUG) Serial.println("Core 1: MPU9250 calibration complete.");
  
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(1000);

  // --- TASK LOOP ---
  for (;;) {
    xyzFloat gyr = myMPU9250.getGyrRawValues();

    gx = gyr.x;
    gy = gyr.y;
    gz = gyr.z;

    // Send data over UART2 in binary format
    Serial.write(SYNC_BYTE);
    Serial.write((uint8_t *)&gx, sizeof(gx));
    Serial.write((uint8_t *)&gy, sizeof(gy));
    Serial.write((uint8_t *)&gz, sizeof(gz));

    // A small delay to prevent this task from hogging the core completely
    vTaskDelay(1); 
    // taskYIELD();
  }
}


//================================================================================
//                                  SETUP
//================================================================================
void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(1500000);
  delay(2000);
  if (SERIAL_DEBUG) Serial.println("--- Dual Core Sensor Demo ---");

  // Initialize the two data UARTs with their respective TX pins
  Serial1.begin(1500000, SERIAL_8N1, -1, UART1_TX_PIN); // RX pin -1 (unused)

  hspi.begin();
  pinMode(HSPI_CS_PIN, OUTPUT);

  // Create the IIS3DWB task and pin it to Core 0
  xTaskCreatePinnedToCore(
      taskIIS,      // Function to implement the task
      "IIS_Task",   // Name of the task
      10000,        // Stack size in words
      NULL,         // Task input parameter
      1,            // Priority of the task (1 is low)
      &taskIIS_Core0, // Task handle
      0);           // Pin task to Core 0

  // Create the MPU9250 task and pin it to Core 1
  xTaskCreatePinnedToCore(
      taskMPU,      // Function to implement the task
      "MPU_Task",   // Name of the task
      10000,        // Stack size in words
      NULL,         // Task input parameter
      1,            // Priority of the task
      &taskMPU_Core1, // Task handle
      1);           // Pin task to Core 1

  if (SERIAL_DEBUG) Serial.println("Setup complete. Tasks are running.");
}

//================================================================================
//                                   LOOP
//================================================================================
void loop() {
  // The main loop() runs on Core 1 by default, but our tasks handle all the work.
  // This can be left empty or used for low-priority, non-blocking code.
  delay(1000);
}
