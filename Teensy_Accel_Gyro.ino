#include "IIS3DWB.h"     // Library for IIS3DWB sensor
#include "SPI.h"          // SPI Protocol
#include "MPU9250_WE.h"  // Library for MPU9250 sensor

#define CSPIN 10       // CS pin for IIS3DWB
#define MPU_CS_PIN 38  // CS pin for MPU9250
#define DIG_PWR_CNTL 4 // Power control for IIS3DWB

// Sync bytes for framing telemetry
const int8_t SYNC_BYTE   = 0xAA;

// --- Instantiate sensors on correct SPI buses ---
IIS3DWB IIS3DWB(CSPIN);    // IIS3DWB on SPI0
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI1, MPU_CS_PIN, 26, 39, 27, true);  

float aRes;                             
float accelBias[3] = {0, 0, 0};
float Gyro_bias[3] = {0, 0, 0};

const float acc_mult_factor = 1000.0;  // Convert accel to mg
const float gyr_mult_factor = 10.0;    // Scale gyro for precision

int16_t ax, ay, az;
int16_t Gx, Gy, Gz;

unsigned long lastMicros = 0;
volatile bool dataReady = false;

IntervalTimer dataTimer; // 25 kHz execution timer

/* -------------------- Setup Functions -------------------- */
void IIS3DWB_SETUP() {
  // Ensure proper power-up sequence
  digitalWrite(DIG_PWR_CNTL, LOW);
  delay(100);
  digitalWrite(DIG_PWR_CNTL, HIGH);
  delay(100);
  
  IIS3DWB.reset();    
  delay(100);  // Increased delay for proper reset
  
  // Initialize with proper configuration
  IIS3DWB.init(AFS_8G);
  delay(50);
  
  // Get resolution after initialization
  aRes = IIS3DWB.getAres(AFS_8G);
  
  // Set bias after everything is initialized
  IIS3DWB.offsetBias(accelBias);
  
  Serial.print("IIS3DWB aRes: ");
  Serial.println(aRes, 6);
}

uint8_t IIS3DWB_CHECK() {
  uint8_t chipID = IIS3DWB.getChipID();
  Serial.print("IIS3DWB Chip ID: 0x");
  Serial.println(chipID, HEX);
  return chipID;
}

void calibrate_MPU(float gyro_bias[]) {
  const int samples = 2000;
  const int delay_us = 1000; // Increased delay for stability

  Serial.println("Calibrating MPU9250 gyroscope...");
  
  for (int i = 0; i < 3; i++) gyro_bias[i] = 0.0;

  for (int i = 0; i < samples; i++) {
    xyzFloat gyr = myMPU9250.getGyrValues();
    gyro_bias[0] += gyr.x;
    gyro_bias[1] += gyr.y;
    gyro_bias[2] += gyr.z;
    delayMicroseconds(delay_us);
    
    if (i % 500 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print((i * 100) / samples);
      Serial.println("%");
    }
  }
  
  for (int i = 0; i < 3; i++) gyro_bias[i] /= samples;
  
  Serial.println("Gyro calibration complete:");
  Serial.print("X bias: "); Serial.println(gyro_bias[0]);
  Serial.print("Y bias: "); Serial.println(gyro_bias[1]);
  Serial.print("Z bias: "); Serial.println(gyro_bias[2]);
}

/* -------------------- Data Capture -------------------- */
void iis3dwb_data() {
  // Check if data is ready and chip ID is correct
  uint8_t status = IIS3DWB.DRstatus();
  
  if ((status & 0x01)) {  // Data ready bit
    int16_t raw[3];
    IIS3DWB.readAccelData(raw);
    
    // Apply proper scaling and bias correction
    ax = ((raw[0] * aRes - accelBias[0]) * acc_mult_factor);
    ay = (int16_t)((raw[1] * aRes - accelBias[1]) * acc_mult_factor);
    az = (int16_t)((raw[2] * aRes - accelBias[2]) * acc_mult_factor);
  } else {
    ax = ay = az = -7777; // Data not ready marker
  }
}

void data_collection() {
  if (dataReady) return; // Prevent re-entrance
  dataReady = true;
  
  lastMicros = micros();

  // Read IIS3DWB accelerometer data
  iis3dwb_data();
  
  // Read MPU9250 gyroscope data
  xyzFloat gyr = myMPU9250.getGyrValues();

  Gx = (int16_t)((gyr.x - Gyro_bias[0]) * gyr_mult_factor);
  Gy = (int16_t)((gyr.y - Gyro_bias[1]) * gyr_mult_factor);
  Gz = (int16_t)((gyr.z - Gyro_bias[2]) * gyr_mult_factor);

  // ---------- Telemetry Packet ----------
  Serial.write(SYNC_BYTE);
  // Serial.write(SYNC_BYTE_1);  // Uncomment if needed

  // Serial.write((uint8_t*)&lastMicros, sizeof(lastMicros)); // Uncomment if needed
  Serial.write((uint8_t*)&ax, sizeof(ax));
  Serial.write((uint8_t*)&ay, sizeof(ay));
  Serial.write((uint8_t*)&az, sizeof(az));

  Serial.write((uint8_t*)&Gx, sizeof(Gx));
  Serial.write((uint8_t*)&Gy, sizeof(Gy));
  Serial.write((uint8_t*)&Gz, sizeof(Gz));
  
  dataReady = false;
}

/* -------------------- Setup & Loop -------------------- */
void setup() {
  // Initialize power control
  pinMode(CSPIN, OUTPUT);
  digitalWrite(CSPIN, HIGH);
  pinMode(MPU_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH);
  pinMode(DIG_PWR_CNTL, OUTPUT);
  digitalWrite(DIG_PWR_CNTL, HIGH); 

  Serial.begin(1500000);  
  delay(2000); // Allow serial to initialize

  Serial.println("Initializing sensors...");

  // --- Init SPI0 for IIS3DWB ---
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1MHz for initialization
  SPI.endTransaction();

  // --- Init SPI1 for MPU9250 ---
  SPI1.setMOSI(26);
  SPI1.setMISO(39);
  SPI1.setSCK(27);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1MHz for initialization
  SPI1.endTransaction();

  delay(100);

  // Init MPU9250 first
  Serial.println("Initializing MPU9250...");
  if (myMPU9250.init()) {
    Serial.println("MPU9250 initialized successfully");
    myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800);
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
    myMPU9250.setMagOpMode(AK8963_PWR_DOWN);
  } else {
    Serial.println("MPU9250 initialization failed!");
  }

  // Init IIS3DWB
  Serial.println("Checking IIS3DWB...");
  uint8_t chipID = IIS3DWB_CHECK();
  if (chipID == 0x7B) {
    Serial.println("IIS3DWB detected, initializing...");
    IIS3DWB_SETUP();
    Serial.println("IIS3DWB initialized successfully");
  } else {
    Serial.print("IIS3DWB not detected! Got ID: 0x");
    Serial.println(chipID, HEX);
    Serial.println("Expected: 0x7B");
  }

  // Calibrate gyroscope
  calibrate_MPU(Gyro_bias);

  Serial.println("Starting data collection...");
  delay(1000);

  // Start data collection @ 25 kHz (40 µs)
  dataTimer.begin(data_collection, 40);
}

void loop() {
  // Debug output every 2 seconds
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    
    // Temporarily disable timer for debug
    dataTimer.end();
    
    Serial.print("Debug - Accel: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az);
    Serial.print(" | Gyro: ");
    Serial.print(Gx); Serial.print(", ");
    Serial.print(Gy); Serial.print(", ");
    Serial.println(Gz);
    
    Serial.print("IIS3DWB Status: 0x");
    Serial.print(IIS3DWB.DRstatus(), HEX);
    Serial.print(" | Chip ID: 0x");
    Serial.println(IIS3DWB.getChipID(), HEX);
    
    // Re-enable timer
    dataTimer.begin(data_collection, 40);
  }
}