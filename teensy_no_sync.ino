#define SYNC_BYTE 0xAA
#define BAUD_RATE 1500000  // Must match ESP32

int16_t Ax = 0, Ay = 0, Az = 0;
int16_t Gx = 0, Gy = 0, Gz = 0;

unsigned long loopInterval = 40;
unsigned long lastMicros;

void setup() {
  Serial.begin(1500000);   // USB Serial for Telemetry
  Serial1.begin(BAUD_RATE); // GYRO on Serial1
  Serial2.begin(BAUD_RATE); // ACCEL on Serial2

  Serial.println("Teensy Started...");
  delay(2000);
}

void loop() 
{
  if(micros() - lastMicros >= loopInterval)
  {
    lastMicros = micros();

    byte recvData_g[7];
    if (Serial1.available() >= (int)sizeof(recvData_g))
    {
      Serial1.readBytes(recvData_g, sizeof(recvData_g)); // Read the whole chunk
        if (recvData_g[0] == SYNC_BYTE) 
        {
            memcpy(&Gx, &recvData_g[1], sizeof(Gx)); // Extract Gx
            memcpy(&Gy, &recvData_g[3], sizeof(Gy)); // Extract Gy
            memcpy(&Gz, &recvData_g[5], sizeof(Gz)); // Extract Gz
        }
    }


    byte recvData_a[7];
    if (Serial2.available() >= (int)sizeof(recvData_a))
    {
      Serial2.readBytes(recvData_a, sizeof(recvData_a)); // Read the whole chunk
        if (recvData_a[0] == SYNC_BYTE) 
        {
            memcpy(&Ax, &recvData_a[1], sizeof(Ax)); // Extract Ax
            memcpy(&Ay, &recvData_a[3], sizeof(Ay)); // Extract Ay
            memcpy(&Az, &recvData_a[5], sizeof(Az)); // Extract Az
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
