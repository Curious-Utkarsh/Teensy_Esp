#include "esp_task_wdt.h"
#include <MPU9250_WE.h>
#include "SPI.h"

const int8_t SYNC_BYTE = 0xAA;
int16_t Gx{ 0 };
int16_t Gy{ 0 };
int16_t Gz{ 0 };

const int csPin = 1; //CS Pin
const int mosiPin = 9;  // "MOSI" Pin
const int misoPin = 8;  // "MISO" Pin
const int sckPin = 7;  // SCK Pin
bool useSPI = true;    // SPI use flag

MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, csPin, mosiPin, misoPin, sckPin, useSPI);

void setup() {

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  Serial.begin(1500000);
  Serial1.begin(1500000, SERIAL_8N1, 43, 44);
  delay(2000); // Allow serial to initialize

  //Serial.println("Initializing sensors...");

  // --- Init SPI0 for MPU9250 ---
  // For ESP32, you define the pins inside the begin() function
  SPI.begin(sckPin, misoPin, mosiPin);
  
  // The beginTransaction() and endTransaction() calls here are not necessary
  // for initialization. The library will handle transactions.
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); 
  // SPI.endTransaction();
  
  esp_task_wdt_delete(NULL);

  myMPU9250.init();
  delay(1000);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);  // bandwdith without DLPF
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(1000);
}

void loop() {
    xyzFloat gyr = myMPU9250.getGyrRawValues();

    Gx = gyr.x;
    Gy = gyr.y;
    Gz = gyr.z;

    Serial1.write(0xAA); // Send the start/sync byte

    Serial1.write((uint8_t*)&(Gx), sizeof(Gx));
    Serial1.write((uint8_t*)&(Gy), sizeof(Gy));
    Serial1.write((uint8_t*)&(Gz), sizeof(Gz));
}