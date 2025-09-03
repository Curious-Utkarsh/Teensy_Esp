#define SYNC_BYTE 0xAA
#define BAUD_RATE 1500000  // Must match ESP32 UART rate

int16_t Ax = 0, Ay = 0, Az = 0;
int16_t Gx = 0, Gy = 0, Gz = 0;

bool accel_received = false; //GPIO Pin 0
bool gyro_received = false;  //GPIO Pin 7


void setup() {
  Serial.begin(1500000);     // Telemetry USB serial
  Serial1.begin(BAUD_RATE);  // From MPU9250 ESP32
  Serial2.begin(BAUD_RATE);  // From IIS3DWB ESP32
}

void loop() {
  // Handle Serial1 (GYRO)
  if (Serial1.available() >= 7)
  {
    if(Serial1.read() == SYNC_BYTE)  // Consume SYNC_BYTE
    {
      Serial1.readBytes((char*)&Gx, sizeof(Gx));
      Serial1.readBytes((char*)&Gy, sizeof(Gy));
      Serial1.readBytes((char*)&Gz, sizeof(Gz));
      gyro_received = true;
    }
  }

  // Handle Serial2 (ACCEL)
  if (Serial2.available() >= 7) 
  {
    if(Serial2.read() == SYNC_BYTE)  // Consume SYNC_BYTE
    {
      Serial2.readBytes((char*)&Ax, sizeof(Ax));
      Serial2.readBytes((char*)&Ay, sizeof(Ay));
      Serial2.readBytes((char*)&Az, sizeof(Az));
      accel_received = true;
    }
  }

  // Send combined data only if both are received
  if (accel_received && gyro_received) 
  {
    Serial.write(SYNC_BYTE);  // Start byte
    
    Serial.write((uint8_t*)&Ax, sizeof(Ax));
    Serial.write((uint8_t*)&Ay, sizeof(Ay));
    Serial.write((uint8_t*)&Az, sizeof(Az));

    Serial.write((uint8_t*)&Gx, sizeof(Gx));
    Serial.write((uint8_t*)&Gy, sizeof(Gy));
    Serial.write((uint8_t*)&Gz, sizeof(Gz));

    accel_received = false;
    gyro_received = false;
  }
}
