//Teensy Data Accept
#define SYNC_BYTE 0xAA
#define BAUD_RATE 1500000  // Must match ESP32

#define CLK_PIN 3

volatile bool accel_ = false;
volatile bool gyro_ = false;

int16_t Ax = 0, Ay = 0, Az = 0;
int16_t Gx = 0, Gy = 0, Gz = 0;

void setup() {
  Serial.begin(1500000);   // USB Serial for Telemetry
  Serial1.begin(BAUD_RATE); // GYRO on Serial1
  Serial2.begin(BAUD_RATE); // ACCEL on Serial2
  pinMode(CLK_PIN, OUTPUT);
}

void loop() {
  digitalWrite(CLK_PIN, HIGH);
  delayMicroseconds(1);'00'uu
  // Read ACCEL data RISING EDGE
  if (Serial2.available() > 0 && Serial2.read() == SYNC_BYTE) {
    if (Serial2.available() < 12) {  // Ensure we have full packet
      Serial2.readBytes((char*)&Ax, sizeof(Ax));
      Serial2.readBytes((char*)&Ay, sizeof(Ay));
      Serial2.readBytes((char*)&Az, sizeof(Az));
      accel_ = true;  // Flag that accel data is received
    }
  }
  //Check or Not??
  digitalWrite(CLK_PIN, LOW);
  delayMicroseconds(1);
  // Read GYRO data FALLING EDGE
  if (Serial1.available() > 0 && Serial1.read() == SYNC_BYTE) {
    if (Serial1.available() < 12) {  // Ensure we have full packet
      Serial1.readBytes((char*)&Gx, sizeof(Gx));
      Serial1.readBytes((char*)&Gy, sizeof(Gy));
      Serial1.readBytes((char*)&Gz, sizeof(Gz));
      gyro_ = true;  // Flag that gyro data is received
    }
  }

  if(gyro_ == true && accel_ == true)
  {
    Serial.write(SYNC_BYTE);  // Send sync byte

    Serial.write((uint8_t*)&Ax, sizeof(Ax));
    Serial.write((uint8_t*)&Ay, sizeof(Ay));
    Serial.write((uint8_t*)&Az, sizeof(Az));

    Serial.write((uint8_t*)&Gx, sizeof(Gx));
    Serial.write((uint8_t*)&Gy, sizeof(Gy));
    Serial.write((uint8_t*)&Gz, sizeof(Gz));

    gyro_ = false;
    accel_ = false;
  }

}








