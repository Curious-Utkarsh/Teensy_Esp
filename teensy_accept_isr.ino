//Teensy Data Accept
#define SYNC_BYTE 0xAA
#define BAUD_RATE 1500000  // Must match ESP32

#define cts_g    3
#define valid_g  4

#define cts_a    5
#define valid_a  6

int16_t Ax = 0, Ay = 0, Az = 0;
int16_t Gx = 0, Gy = 0, Gz = 0;

unsigned long dt = 50;
unsigned long lastMicros;

void setup() {
  Serial.begin(1500000);   // USB Serial for Telemetry
  Serial1.begin(BAUD_RATE); // GYRO on Serial1
  Serial2.begin(BAUD_RATE); // ACCEL on Serial2

  pinMode(cts_g, OUTPUT);
  pinMode(valid_g, INPUT);

  pinMode(cts_a, OUTPUT);
  pinMode(valid_a, INPUT);

  Serial.println("Teensy Started...");
  delay(2000);
}

void loop() 
{
  lastMicros = 0;
  while(1)
  {
    if(micros() - lastMicros >= dt)
    {
      lastMicros = micros();

      //Waiting for Valid Data Signal
      while(digitalRead(valid_g) == 0);
      //Signal High Recieved, now setting CTS as high
      digitalWrite(cts_g, HIGH);
      //Now Waiting for Serial Data
      while(!Serial1.available());
      //Data recieved, put cts as low
      digitalWrite(cts_g, LOW);
      //Data Got avaiable, stored in buffer for reading later.

      //Waiting for Valid Data Signal
      while(digitalRead(valid_a) == 0);
      //Signal High Recieved, now setting CTS as high
      digitalWrite(cts_a, HIGH);
      //Now Waiting for Serial Data
      while(!Serial2.available());
      //Data recieved, put cts as low
      digitalWrite(cts_a, LOW);
      //Data Got avaiable, stored in buffer for reading later.

      //Now Reading from Serial1 Buffer and Serial2 Buffer.
      if (Serial1.read() == SYNC_BYTE) 
      {
        Serial1.readBytes((char*)&Gx, sizeof(Gx));
        Serial1.readBytes((char*)&Gy, sizeof(Gy));
        Serial1.readBytes((char*)&Gz, sizeof(Gz));
      }
      if (Serial2.read() == SYNC_BYTE) 
      {
        Serial2.readBytes((char*)&Ax, sizeof(Ax));
        Serial2.readBytes((char*)&Ay, sizeof(Ay));
        Serial2.readBytes((char*)&Az, sizeof(Az));
      }

      //Now Writing to Telemetry
      Serial.write(SYNC_BYTE);
      Serial.write((uint8_t*)&Ax, sizeof(Ax));
      Serial.write((uint8_t*)&Ay, sizeof(Ay));
      Serial.write((uint8_t*)&Az, sizeof(Az));
      Serial.write((uint8_t*)&Gx, sizeof(Gx));
      Serial.write((uint8_t*)&Gy, sizeof(Gy));
      Serial.write((uint8_t*)&Gz, sizeof(Gz));
    }
  }
}
  
