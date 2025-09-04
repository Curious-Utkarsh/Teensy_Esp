/**
 * Core 0: IIS3DWB -> VSPI -> UART1 (TX=GPIO17)
 * Core 1: MPU9250 -> HSPI -> USB Serial0
 */

#include <Arduino.h>
#include <SPI.h>
#include "IIS3DWB.h"
#include <MPU9250_WE.h>
#include "esp_task_wdt.h"

// -------------------- CONFIG --------------------
#define SERIAL_DEBUG false

// VSPI (IIS3DWB)
#define VSPI_CS_PIN   5
#define VSPI_MOSI_PIN 23
#define VSPI_MISO_PIN 19
#define VSPI_SCK_PIN  18
#define UART1_TX_PIN  17

// HSPI (MPU9250)
#define HSPI_CS_PIN   15
#define HSPI_MOSI_PIN 13
#define HSPI_MISO_PIN 12
#define HSPI_SCK_PIN  14

// Packet sync
const uint8_t SYNC_BYTE = 0xAA;

// IIS3DWB config/state
IIS3DWB iis3dwb(VSPI_CS_PIN);
const uint8_t Ascale = AFS_2G;
float aRes = 0.0f;
float accelBias[3] = {0.0f, 0.0f, 0.0f};
int16_t iis3dwbData[3] = {0};
const float ACC_MULT_FACTOR = 1000.0f;
int16_t garbageValue = -5000;

// HSPI bus + MPU9250
SPIClass hspi(HSPI);
MPU9250_WE myMPU9250 = MPU9250_WE(&hspi, HSPI_CS_PIN, HSPI_MOSI_PIN, HSPI_MISO_PIN, HSPI_SCK_PIN, true);

// -------------------- RTOS HANDLES --------------------
TaskHandle_t taskIIS_Core0;
TaskHandle_t taskMPU_Core1;

// Mutexes (for race-free access)
SemaphoreHandle_t serial0Mutex;  // USB Serial
SemaphoreHandle_t serial1Mutex;  // UART1
SemaphoreHandle_t vspiMutex;     // VSPI bus
SemaphoreHandle_t hspiMutex;     // HSPI bus

// Helper macros
#define LOCK(m)   xSemaphoreTake((m), portMAX_DELAY)
#define UNLOCK(m) xSemaphoreGive((m))

// ================= CORE 0: IIS3DWB TASK =================
void taskIIS(void *pvParameters) {
  if (SERIAL_DEBUG) { LOCK(serial0Mutex); Serial.printf("IIS3DWB task on core %d\n", xPortGetCoreID()); UNLOCK(serial0Mutex); }

  // Init VSPI (global SPI)
  SPI.begin(VSPI_SCK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN);
  pinMode(VSPI_CS_PIN, OUTPUT);
  digitalWrite(VSPI_CS_PIN, HIGH);

  // Sensor init — protect VSPI access
  LOCK(vspiMutex);
  iis3dwb.reset();
  UNLOCK(vspiMutex);
  delay(100);

  uint8_t chipID;
  LOCK(vspiMutex);
  chipID = iis3dwb.getChipID();
  UNLOCK(vspiMutex);

  if (chipID == 0x7B) {
    if (SERIAL_DEBUG) { LOCK(serial0Mutex); Serial.println("Core0: IIS3DWB OK"); UNLOCK(serial0Mutex); }
    LOCK(vspiMutex);
    aRes = iis3dwb.getAres(Ascale);
    iis3dwb.init(Ascale);
    iis3dwb.offsetBias(accelBias);
    UNLOCK(vspiMutex);
  } else {
    if (SERIAL_DEBUG) { LOCK(serial0Mutex); Serial.printf("Core0: IIS3DWB not found (0x%02X)\n", chipID); UNLOCK(serial0Mutex); }
  }

  // Loop
  for (;;) {
    uint8_t packet[7]; // [SYNC][ax][ay][az]
    packet[0] = SYNC_BYTE;

    if (chipID == 0x7B) {
      // Read data (guard VSPI + sensor lib)
      bool haveData = false;
      LOCK(vspiMutex);
      uint8_t dr = iis3dwb.DRstatus();
      if (dr & 0x01) {
        iis3dwb.readAccelData(iis3dwbData);
        haveData = true;
      }
      UNLOCK(vspiMutex);

      if (haveData) {
        int16_t ax = ACC_MULT_FACTOR * ((float)iis3dwbData[0] * aRes - accelBias[0]);
        int16_t ay = ACC_MULT_FACTOR * ((float)iis3dwbData[1] * aRes - accelBias[1]);
        int16_t az = ACC_MULT_FACTOR * ((float)iis3dwbData[2] * aRes - accelBias[2]);

        memcpy(&packet[1], &ax, 2);
        memcpy(&packet[3], &ay, 2);
        memcpy(&packet[5], &az, 2);

        // UART1 write is not shared, but still protect for safety
        LOCK(serial1Mutex);
        if (Serial1.availableForWrite() >= sizeof(packet)) {
            Serial1.write(packet, sizeof(packet));
        }
        UNLOCK(serial1Mutex);
      }
    } else {
      memcpy(&packet[1], &garbageValue, 2);
      memcpy(&packet[3], &garbageValue, 2);
      memcpy(&packet[5], &garbageValue, 2);
      LOCK(serial1Mutex);
      if (Serial1.availableForWrite() >= sizeof(packet)) {
          Serial1.write(packet, sizeof(packet));
      }
      UNLOCK(serial1Mutex);
    }

    // Optional: cooperate without adding fixed latency
    taskYIELD();
  }
}

// ================= CORE 1: MPU9250 TASK =================
void taskMPU(void *pvParameters) {
  if (SERIAL_DEBUG) { LOCK(serial0Mutex); Serial.printf("MPU9250 task on core %d\n", xPortGetCoreID()); UNLOCK(serial0Mutex); }

  // Avoid WDT during long init
  esp_task_wdt_delete(NULL);

  // Init HSPI
  hspi.begin(HSPI_SCK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN, HSPI_CS_PIN);
  pinMode(HSPI_CS_PIN, OUTPUT);

  // Sensor init guarded by HSPI mutex
  LOCK(hspiMutex);
  bool ok = myMPU9250.init();
  UNLOCK(hspiMutex);

  if (!ok) {
    if (SERIAL_DEBUG) {LOCK(serial0Mutex); Serial.println("Core1: MPU9250 not found"); UNLOCK(serial0Mutex);}
    vTaskDelete(NULL);
  }

  if (SERIAL_DEBUG) { LOCK(serial0Mutex); Serial.println("Core1: MPU9250 init..."); UNLOCK(serial0Mutex);}

  LOCK(hspiMutex);
  myMPU9250.autoOffsets();
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(false);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  UNLOCK(hspiMutex);

  // Loop
  for (;;) {
    xyzFloat gyr;
    LOCK(hspiMutex);
    gyr = myMPU9250.getGyrRawValues();
    UNLOCK(hspiMutex);

    int16_t gx = (int16_t)gyr.x;
    int16_t gy = (int16_t)gyr.y;
    int16_t gz = (int16_t)gyr.z;

    uint8_t packet[7]; // [SYNC][gx][gy][gz]
    packet[0] = SYNC_BYTE;
    memcpy(&packet[1], &gx, 2);
    memcpy(&packet[3], &gy, 2);
    memcpy(&packet[5], &gz, 2);

    // USB Serial is shared with debug prints → protect
    LOCK(serial0Mutex);
    if (Serial.availableForWrite() >= sizeof(packet)) {
        Serial.write(packet, sizeof(packet));
    }
    UNLOCK(serial0Mutex);

    taskYIELD();
  }
}

// ========================== SETUP ==========================
void setup() {
  // Create mutexes BEFORE any task starts
  serial0Mutex = xSemaphoreCreateMutex();
  serial1Mutex = xSemaphoreCreateMutex();
  vspiMutex    = xSemaphoreCreateMutex();
  hspiMutex    = xSemaphoreCreateMutex();


  // Start serials
  Serial.begin(1500000);
  Serial1.begin(1500000, SERIAL_8N1, -1, UART1_TX_PIN);

  if (SERIAL_DEBUG) { LOCK(serial0Mutex); Serial.println("--- Dual Core (Race-Safe) ---"); UNLOCK(serial0Mutex); }

  // Create tasks pinned to different cores
  xTaskCreatePinnedToCore(taskIIS, "IIS_Task", 10000, NULL, 2, &taskIIS_Core0, 0);
  xTaskCreatePinnedToCore(taskMPU, "MPU_Task", 10000, NULL, 2, &taskMPU_Core1, 1);
}

void loop() {
  // Idle — all work handled in tasks
  //delay(1000);
}
