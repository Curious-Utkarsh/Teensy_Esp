#include "IIS3DWB.h"
#include "SPI.h"
#include <Arduino.h>

// SPI & IIS3DWB setup
#define CSPIN    5       // IIS3DWB Chip Select Pin
#define TX_PIN   17      // ESP32 TX1 (UART1)
#define RX_PIN   16      // ESP32 RX1 (Not Used)
#define SYNC_BYTE 0xAA   // Sync Byte to identify data packets

// IIS3DWB Sensor Variables
uint8_t Ascale = AFS_2G;
float aRes;
int16_t IIS3DWBData[3] = {0};
int16_t Ax, Ay, Az;
const float acc_mult_factor = 1000.0;
IIS3DWB IIS3DWB(CSPIN);

unsigned long startTime = 0;
unsigned long loopInterval = 30; // 1ms for 1kHz loop

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

    Serial.println("ESP Started...");
    delay(2000);
}

void loop() 
{
    if (micros() - startTime >= loopInterval) {
        startTime = micros();

        // Data Read from Sensor
        if (IIS3DWB.DRstatus() & 0x01) {
            IIS3DWB.readAccelData(IIS3DWBData);
            Ax = acc_mult_factor * (IIS3DWBData[0] * aRes);
            Ay = acc_mult_factor * (IIS3DWBData[1] * aRes);
            Az = acc_mult_factor * (IIS3DWBData[2] * aRes);
        }

        // Ax = (Ax>4000)? 0 : ++Ax;
        // Ay = (Ay>5000)? 0 : ++Ay;
        // Az = (Az>6000)? 0 : ++Az;

        byte sendData[7];
        sendData[0] = SYNC_BYTE;
        memcpy(&sendData[1], &Ax, sizeof(Ax));  // Copy ax to sendData
        memcpy(&sendData[3], &Ay, sizeof(Ay));  // Copy ay to sendData
        memcpy(&sendData[5], &Az, sizeof(Az));  // Copy az to sendData

        // Send all data in one go
        Serial2.write(sendData, sizeof(sendData));
    }
}

