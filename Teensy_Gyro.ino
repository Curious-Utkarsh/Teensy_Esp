#include "SPI.h"        // Library for SPI Protocol
#include "MPU9250_WE.h" // Library for MPU9250 sensor

// Define the Chip Select pin for the MPU9250
// #define MPU_CS_PIN 10

#define MPU_CS_PIN 38

// --- Global Variables ---

// Define sync bytes for robust serial communication framing
const int8_t SYNC_BYTE = 0xAA;

// Multiplier to convert float sensor values to int16_t for transmission
const float acc_mult_factor = 1000.0; // convert g to milli-g
const float gyr_mult_factor = 10.0;   // Increase precision for gyro

// Variables to hold the final sensor data
int16_t Gx = 0, Gy = 0, Gz = 0;

// MPU9250 sensor object instance using the default SPI bus
//MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI1, 38, 26, 39, 27, true);

// IntervalTimer for precise data sampling rate
IntervalTimer dataTimer;


// --- Data Collection (Interrupt Service Routine) ---

/**
 * @brief This function is called by the IntervalTimer at a fixed frequency.
 * It reads sensor data and sends it over the serial port.
 * Using an interrupt ensures consistent timing, independent of other code.
 */
void data_collection() {
  unsigned long currentMicros = micros();

  // Read calibrated sensor data.
  // getAccValues() and getGyrValues() return calibrated data based on autoOffsets().
  xyzFloat gyr = myMPU9250.getGyrValues();

  // Convert float values to scaled int16_t for efficient serial transmission
  Gx = gyr.x * gyr_mult_factor;
  Gy = gyr.y * gyr_mult_factor;
  Gz = gyr.z * gyr_mult_factor;


  // --- Serial Data Transmission ---
  // Data is sent in a binary packet format with sync bytes for framing.
  // This is much more efficient than sending ASCII text.
  Serial.write(SYNC_BYTE);
  Serial.write((uint8_t *)&Gx, sizeof(Gx)); // Gyro X
  Serial.write((uint8_t *)&Gy, sizeof(Gy)); // Gyro Y
  Serial.write((uint8_t *)&Gz, sizeof(Gz)); // Gyro Z
}


// --- Main Setup and Loop ---

void setup() {
  // Start serial communication for data output at a high baud rate
  Serial.begin(1500000);

  // Configure and begin the SPI bus for the MPU9250
  // SPI.setMOSI(11);
  // SPI.setMISO(12);
  // SPI.setSCK(13);
  // SPI.begin();

  SPI1.setMOSI(26);
  SPI1.setMISO(39);
  SPI1.setSCK(27);
  SPI1.begin();

  myMPU9250.init();
  delay(1000);

  // --- Sensor Configuration ---
  // Perform automatic calibration of accelerometer and gyroscope.
  // The sensor should be kept still and level during this process.
  myMPU9250.autoOffsets();

  // Apply sensor settings from your new code
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // Disable Gyro DLPF
  myMPU9250.enableAccDLPF(true);                     // Enable Accel DLPF
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);              // Set Accel DLPF bandwidth
  myMPU9250.setMagOpMode(AK8963_PWR_DOWN);           // Power down magnetometer as it's not used

  delay(1000);

  // Start the IntervalTimer to call data_collection every 40 microseconds.
  // This creates a sampling frequency of 25 kHz (1,000,000 / 40 = 25,000).
  dataTimer.begin(data_collection, 40);
}

/*
  The main loop is empty because all the work is handled by the
  data_collection() function, which is triggered by an IntervalTimer interrupt.
  This is a more efficient and reliable method for high-frequency tasks.
*/
void loop() {
  // Intentionally left blank.
}
