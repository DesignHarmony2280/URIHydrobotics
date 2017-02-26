/*
 * Hydrobotics Team
 * University of Rhode Island
 * 
 * ROV CONTROLLER
 * Rev 0.5.5 - ESW Conference Display
 * System intended for the Arduino Mega 2560
 * 
 * * REV DOC -
 * ----------------------------------------------------------
 * Rev 0.1 - Test of the original Send & Recieve code
 * 
 * Rev 0.5 - Proof of Concept System. Displays the 
 * X, Y, and Z components of the triple axis joysticks
 * on the ROV Mega from the TOPSIDE mega over CAN.
 * 
 * Rev 0.5.5 - ESW Conference display. Shows off the 
 * manipulator arm with the 0.5 code for both joysticks.
 * The manipulator at this time implemented a bi-directional
 * DC motor, and an H bridge with two inputs A & B for 
 * forward and reverse pwm speed modulation.
 * ----------------------------------------------------------
 * 
 * Topside = Yellow Mega
 * ROV = unmarked
 * 
 */

#include <LiquidCrystal.h>
#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 9; // CAN CS pin

int backl = 6;
int manipB = 22;
int manipF = 23;

boolean closed = true;
boolean working = false;

unsigned long Timer = 0;

LiquidCrystal lcd1 (12,11,5,4,3,2);

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    pinMode(backl, OUTPUT);     // Backlight
    pinMode(manipB, OUTPUT);    // Manipulator move backward
    pinMode(manipF, OUTPUT);    // Manipulator move forward
    
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    
    lcd1.begin(20, 4); 

    delay(10);
}


void loop()
{
    analogWrite(backl, HIGH);
    
    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        
        unsigned int canId = CAN.getCanId();

        lcd1.setCursor(0,1);
        lcd1.print("x1: ");
        lcd1.print(buf[0]);
        lcd1.setCursor(10,1);
        lcd1.print("x2: ");
        lcd1.print(buf[3]);
        
        lcd1.setCursor(0,2);
        lcd1.print("y1: ");
        lcd1.print(buf[1]);
        lcd1.setCursor(10,2);
        lcd1.print("y2: ");
        lcd1.print(buf[4]);
        
        lcd1.setCursor(0,4);
        lcd1.print("z1: ");
        lcd1.print(buf[2]);
        lcd1.setCursor(10,4);
        lcd1.print("z2: ");
        lcd1.print(buf[5]);

        lcd1.setCursor(0,0);
        lcd1.print("       CAN BUS");

        int command = buf[6];

        if (command == 1 && closed) {
          Timer = millis();
          digitalWrite(22, HIGH);
          working = true;
          closed = !closed;
        }

        if (command == 2 && !closed) {
          Timer = millis();
          digitalWrite(23, HIGH);
          working = true;
          closed = !closed;
        }

        if (working && millis() - Timer >= 750UL) {
          digitalWrite(22, LOW);
          digitalWrite(23, LOW);
        }
        
        delay(100);
        lcd1.clear();
    }

}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
