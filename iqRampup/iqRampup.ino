//Arduino RX is directly connected to motor TX
//Arduino TX is directly connected to motor RX
#include <iq_module_communication.hpp>


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
IqSerial ser0(Serial);
IqSerial ser1(Serial1);
PropellerMotorControlClient prop(0);
BrushlessDriveClient mot(0);

float motorRadsTestVal = 50; //MAX 600
float motorRads = 0;
int interruptPin = 13; 

void setup() 
{
  ser0.begin(115200);
  ser1.begin(115200);
  ser0.set(prop.timeout_,1000.0f);
  ser1.set(prop.timeout_,1000.0f);
  //delay(1000);

  ramp();
//  float nowRad = 1;
//  if(nowRad<motorRads)
//  {
//    ser0.set(prop.ctrl_velocity_,motorRads);
//    ser1.set(prop.ctrl_velocity_,motorRads);
//    nowRad++;
//  }
}

void loop() 
{

  ser0.set(prop.ctrl_velocity_,motorRads);
  ser1.set(prop.ctrl_velocity_,motorRads);
  delay(25);
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
