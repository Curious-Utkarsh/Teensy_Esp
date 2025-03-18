#include "IIS3DWB.h"
#include "SPI.h"
#include <Arduino.h>

#define CSPIN    5       // IIS3DWB Chip Select Pin
#define TX_PIN   17      // ESP32 TX1 (UART1)
#define RX_PIN   16      // ESP32 RX1 (Not Used)
#define SYNC_BYTE 0xAA   // Sync Byte to identify data packets

#define valid_a  4
#define cts_a    22

// IIS3DWB Sensor Variables
uint8_t Ascale = AFS_2G;
float aRes;
int16_t IIS3DWBData[3] = {0};
int16_t ax, ay, az;
const float acc_mult_factor = 1000.0;
IIS3DWB IIS3DWB(CSPIN);

unsigned int dt = 1000;
unsigned long startTime;

void setup() {
    Serial.begin(115200);   // Debugging
    Serial2.begin(1500000, SERIAL_8N1, RX_PIN, TX_PIN);  // UART1 for Teensy
    SPI.begin(18, 19, 23, 5);  // SPI for IIS3DWB

    pinMode(CSPIN, OUTPUT);
    digitalWrite(CSPIN, HIGH);

    if (IIS3DWB.getChipID() == 0x7B) {
        IIS3DWB.reset();
        delay(1000);
        aRes = IIS3DWB.getAres(Ascale);
        IIS3DWB.init(Ascale);
    } else {
        Serial.println("IIS3DWB not detected!");
        while (1); // Halt if sensor is not found
    }

  pinMode(valid_a, OUTPUT);
  pinMode(cts_a, INPUT);

  Serial.println("ESP Started...");
  delay(2000);
}

void loop() 
{
  //Data Read from Sensor
  if (IIS3DWB.DRstatus() & 0x01) 
  {
    IIS3DWB.readAccelData(IIS3DWBData);
    ax = acc_mult_factor * (IIS3DWBData[0] * aRes);
    ay = acc_mult_factor * (IIS3DWBData[1] * aRes);
    az = acc_mult_factor * (IIS3DWBData[2] * aRes);
  }

  //Data Ready To Send
  digitalWrite(valid_a, HIGH);

  //Waiting for CTS
  startTime = micros();
  while(digitalRead(cts_a) == 0)
  {
    if(micros()-startTime >= dt)
    {
      digitalWrite(valid_a, LOW);
      return;
    }
  }

  //CTS Recieved, Sendinhg Data
  Serial2.write(SYNC_BYTE);          
  Serial2.write((uint8_t*)&ax, sizeof(ax));
  Serial2.write((uint8_t*)&ay, sizeof(ay));
  Serial2.write((uint8_t*)&az, sizeof(az));

  //Data Sent
  digitalWrite(valid_a, LOW);
}




