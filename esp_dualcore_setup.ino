#include "esp_task_wdt.h"
#include <MPU9250_WE.h>
#include <SPI.h>

//MPU Setup - HSPI
const int csPin = 27;  // Chip Select Pin //NCS
const int mosiPin = 13;  // "MOSI" Pin //SDA
const int misoPin = 12;  // "MISO" Pin //ADO
const int sckPin = 14;  // SCK Pin //SCL

const int ledPin = 2;

SPIClass *hspi = new SPIClass(HSPI);
MPU9250_WE myMPU9250 = MPU9250_WE(hspi, csPin, true);

const int8_t SYNC_BYTE = 0xAA;
int16_t Gx{ 0 };
int16_t Gy{ 0 };
int16_t Gz{ 0 };

TaskHandle_t Task0;
TaskHandle_t Task1;

void Task0code(void * parameter);
void Task1code(void * parameter);

void setup() 
{
  Serial.begin(1500000);

  pinMode(ledPin, OUTPUT);

  hspi->begin(sckPin, misoPin, mosiPin, csPin);

  myMPU9250.init();
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  esp_task_wdt_delete(NULL);
  delay(200);

  //create a task that will be executed in the Task0code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
        Task0code, /* Function to implement the task */
        "Task0", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        1,  /* Priority of the task */
        &Task0,  /* Task handle. */
        0); /* Core where the task should run */
  delay(200);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
        Task1code, /* Function to implement the task */
        "Task1", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        1,  /* Priority of the task */
        &Task1,  /* Task handle. */
        1); /* Core where the task should run */
  delay(200);
}

//Task0Code: MPU9250 - Gyro
void Task0code( void * pvParameters )
{
  // Serial.print("Task0 running on core ");
  // Serial.println(xPortGetCoreID());

  for(;;)
  {
    xyzFloat gyr = myMPU9250.getGyrRawValues();

    Gx = gyr.x;
    Gy = gyr.y;
    Gz = gyr.z;

    Serial.write(SYNC_BYTE);
    Serial.write((uint8_t*)&(Gx), sizeof(Gx));
    Serial.write((uint8_t*)&(Gy), sizeof(Gy));
    Serial.write((uint8_t*)&(Gz), sizeof(Gz));

    //vTaskDelay(1 / portTICK_PERIOD_MS);
  } 
}

//Task1code: 
void Task1code( void * pvParameters )
{
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());

  for(;;)
  {
    digitalWrite(ledPin, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(ledPin, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void loop() 
{

}