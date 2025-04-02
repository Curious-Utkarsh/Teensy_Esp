#define SYNC_BYTE 0xAA
#define BAUD_RATE 1500000  // Must match ESP32

int16_t Ax = 0, Ay = 0, Az = 0;
int16_t Gx = 0, Gy = 0, Gz = 0;

unsigned long startTime = 0;
unsigned long loopInterval = 30;

void setup() {
  Serial.begin(1500000);   // USB Serial for Telemetry
  Serial1.begin(BAUD_RATE); // GYRO on Serial1
  Serial2.begin(BAUD_RATE); // ACCEL on Serial2

  Serial.println("Teensy Started...");
  delay(2000);
}

void loop() 
{
  if(micros() - startTime >= loopInterval)
  {
    startTime = micros();

    byte recvData_g[6];
    if (Serial1.available() >= 1)
    {
      if(Serial1.read() == SYNC_BYTE)
      {
        Serial1.readBytes(recvData_g, sizeof(recvData_g)); // Read the whole chunk except Sync Byte
        memcpy(&Gx, &recvData_g[0], sizeof(Gx)); // Extract Gx
        memcpy(&Gy, &recvData_g[2], sizeof(Gy)); // Extract Gy
        memcpy(&Gz, &recvData_g[4], sizeof(Gz)); // Extract Gz
      }
    }

    byte recvData_a[6];
    if (Serial2.available() >= 1)
    {
      if(Serial2.read() == SYNC_BYTE)
      {
        Serial2.readBytes(recvData_a, sizeof(recvData_a)); // Read the whole chunk except Sync Byte
        memcpy(&Ax, &recvData_a[0], sizeof(Ax)); // Extract Ax
        memcpy(&Ay, &recvData_a[2], sizeof(Ay)); // Extract Ay
        memcpy(&Az, &recvData_a[4], sizeof(Az)); // Extract Az
      }
    }

    // Now Writing to Telemetry
    Serial.write(SYNC_BYTE);  
    Serial.write((uint8_t*)&Ax, sizeof(Ax)); 
    Serial.write((uint8_t*)&Ay, sizeof(Ay)); 
    Serial.write((uint8_t*)&Az, sizeof(Az)); 
    Serial.write((uint8_t*)&Gx, sizeof(Gx)); 
    Serial.write((uint8_t*)&Gy, sizeof(Gy)); 
    Serial.write((uint8_t*)&Gz, sizeof(Gz)); 
  }
}
