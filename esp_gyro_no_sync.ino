#include "esp_task_wdt.h"
#include <MPU9250_WE.h>
#include <SPI.h>

// SPI & MPU9250 setup
#define MPU_CS_PIN 5  // Chip Select for MPU9250 (VSPI)
#define TX_PIN 17     // ESP32 TX1 (UART1)
#define RX_PIN 16     // ESP32 RX1 (Not Used Here)
#define SYNC_BYTE 0xAA

int16_t Gx, Gy, Gz;

MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, MPU_CS_PIN, true);  // Using VSPI

unsigned long startTime = 0;
unsigned long loopInterval = 40; // 1ms for 1kHz loop

void setup() {
  Serial.begin(115200);       // USB Serial for debugging
  Serial2.begin(1500000, SERIAL_8N1, RX_PIN, TX_PIN);  // UART1: 1.5Mbps, 8N1 mode
  esp_task_wdt_delete(NULL);
  myMPU9250.init();
  delay(1000);

  myMPU9250.setSPIClockSpeed(4000000);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(1000);

  Serial.println("ESP Started...");
  delay(2000);
}

void loop()
{
  if(micros() - startTime >= loopInterval)
  {
    startTime = micros();

    xyzFloat gyrValue = myMPU9250.getGyrRawValues();
    Gx = gyrValue.x;
    Gy = gyrValue.y;
    Gz = gyrValue.z;

    byte sendData[8];
    sendData[0] = SYNC_BYTE;
    memcpy(&sendData[1], &Gx, sizeof(Gx));  // Copy Gx to sendData
    memcpy(&sendData[3], &Gy, sizeof(Gy));  // Copy Gy to sendData
    memcpy(&sendData[5], &Gz, sizeof(Gz));  // Copy Gz to sendData

    // Send all data in one go
    Serial2.write(sendData, sizeof(sendData));
  }

}
