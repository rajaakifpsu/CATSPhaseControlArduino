#include <Wire.h>
#include <Servo.h>
#include <iq_module_communication.hpp>
#include <math.h>

/* Mega Serial Pins
 * 
 * Serial   0(RX), 1(TX)
 * Serial1 19(RX), 18(TX)
 * Serial2 17(RX), 16(TX)
 * Serial3 15(RX), 14(TX)
 */
 
/* Due Serial Pins
 * 
 * Serial   0(RX), 1(TX)
 * Serial1 19(RX), 18(TX)
 * Serial2 17(RX), 16(TX)
 * Serial3 15(RX), 14(TX)
 */

/* Wiring Diagram 
 * Name - Color - PinNum
 * 
 *  Motor 1: 
 *  - Red - 
 *  - White - 
 * Ground - Black - GND
 *  
 *  Motor 2:
 *  - Red - 
 *  - White - 
 * Ground - Black - GND 
 *  
 *  Switch:
 * Digital - Green - 13
 * Ground - Brown - GND
 * 
 * //Arduino RX is directly connected to motor TX
 * //Arduino TX is directly connected to motor RX
 */
IqSerial ser0(Serial);
IqSerial ser1(Serial1);
PropellerMotorControlClient prop(0);
BrushlessDriveClient mot(0);

const float MAXMOTORRAD = 550;
float motorRadsTestVal = 5; //MAX 600
float motorRads = 0;

int interruptPin = 13; 
int toggle = 0;
int old = 0;

void setup() 
{
  ser0.begin(115200); //115200 Baud Rate
  ser1.begin(115200);
  ser0.set(prop.timeout_,1000.0f);
  ser1.set(prop.timeout_,1000.0f);
  //delay(1000);
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), coast, CHANGE);

  //time = millis(); // Start counting time in milliseconds
//  ser0.set(prop.ctrl_coast_);
//  ser1.set(prop.ctrl_coast_);
  ramp();
  
}

void loop() 
{
  ser0.set(prop.ctrl_velocity_,0.0f);
  //ser1.set(prop.ctrl_velocity_,0.0f);
  //ser0.set(prop.ctrl_velocity_,motorRads);
  ser1.set(prop.ctrl_velocity_,motorRads);

  toggle = digitalRead(interruptPin);                  //reads switch 1 is high and 0 is low
  if(toggle == 1&&old!=toggle)
    {
      coast(); 
    }
  else if(toggle == 0&&old!=toggle)
  {
      ramp();                
  }
  old = toggle;
  
  delay(25);

}

void coast()
{
  
  float nowRad = motorRads;
  
  while(nowRad>0)
  {
    ser0.set(prop.ctrl_velocity_,nowRad);
    ser1.set(prop.ctrl_velocity_,nowRad);
    nowRad--;
    delay(10);
    
  }
  motorRads = 0; 
}

void ramp()
{
  motorRads = motorRadsTestVal;
  float nowRad = 1;
  while(nowRad<motorRads)
  {
    ser0.set(prop.ctrl_velocity_,nowRad);
    ser1.set(prop.ctrl_velocity_,nowRad);
    nowRad++;
    delay(100);
    
  }
}
