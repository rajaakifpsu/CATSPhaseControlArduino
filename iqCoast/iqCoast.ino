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

float motorRads = 250; //MAX 600
//523.6 rad/s =5000 rpm
void setup() 
{
  ser0.begin(115200);
  ser1.begin(115200);
  ser0.set(prop.timeout_,1000.0f);
  ser1.set(prop.timeout_,1000.0f);
  //delay(1000);

  ser0.set(prop.ctrl_velocity_,1.0f);
  ser1.set(prop.ctrl_velocity_,1.0f);
  for(float i = 0; i<motorRads;i++)
  {
    ser0.set(prop.ctrl_velocity_,i);
    ser1.set(prop.ctrl_velocity_,i);
    Serial.println(static_cast<int>(i));
    delay(100);
  }
  delay(10000);
  coast();
  //ser0.set(prop.ctrl_coast_);
  //ser1.set(prop.ctrl_coast_);
}

void loop() 
{
  
  //ser0.set(prop.ctrl_velocity_,motorRads);
  //ser1.set(prop.ctrl_velocity_,motorRads);
  //delay(25);
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
}
