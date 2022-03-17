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

 //--- VARIABLES --------------------------------------------------//
float elapsedTime = 0, Time = 0, timePrev = 0;
float rad_to_deg = 180/PI;
float PID = 0, PID1 = 0;
float updateangle = 0, desired_angle = 0;
float angle = 0;
float est_angle0 = 0, est_angle1 = 0;
float error0 = 0, error1 = 0;
float pid_p_0 = 0, pid_i_0 = 0, pid_d_0 = 0;
float pid_p_1 = 0, pid_i_1 = 0, pid_d_1 = 0;
float est_diff = 0, diff = 0, diff2 = 0;
float time0 = 0, time1 = 0;
float prevPID_0 = 0, prevPID_1 = 0;
float newvel = 0, obs_vel = 0;


//--- GAIN VALUE SET ---------------------------------------------//
double GAIN = 1.1;

//--- PID CONSTANT SET -------------------------------------------//
double kp=1;   //kp = 1 for 100rad/s, kp = 4 for 650rad/s 
double ki=2;   //ki = 2 for 100rad/s, ki = 8 for 650rad/s
double kd=0;   //Derivative was not used


//--- INITIALIZE SERIAL FOR EACH MOTOR FOR COMMUNICATION ---------//
IqSerial ser0(Serial);
IqSerial ser1(Serial1);

//--- INITIALIZE IQ MODULES --------------------------------------//
PropellerMotorControlClient prop(0);
BrushlessDriveClient mot(0);


//--- MOTOR SPEED SET --------------------------------------------//
const float MAXMOTRAD = 550;
float motorRadsTestVal = 50; //MAX 600
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

  Time = millis(); // Start counting time in milliseconds
  
  //pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), coast, CHANGE);

  //time = millis(); // Start counting time in milliseconds
  ramp();
  
}

void loop() 
{
    // PID call for motor 0
    PID_Function(ser0, motorRads, &pid_p_0, &pid_i_0, &pid_d_0, &prevPID_0, &error0, &time0, &est_angle0, 0);
    timePrev = Time;  
    Time = millis();  
    elapsedTime = (Time - timePrev) / 1000; 
    time0 = elapsedTime;
    time1 += elapsedTime;
    
    // PID call for motor 1
    PID_Function(ser1, motorRads, &pid_p_1, &pid_i_1, &pid_d_1, &prevPID_1, &error1, &time1, &est_angle1, 0);
    timePrev = Time;  
    Time = millis();  
    elapsedTime = (Time - timePrev) / 1000; 
    time0 += elapsedTime;
    time1 = elapsedTime;



  toggle = digitalRead(interruptPin);                  //reads switch 1 is high and 0 is low
//  if(toggle == 1&&old!=toggle)
//    {
//      coast(); 
//    }
//  else if(toggle == 0&&old!=toggle)
//  {
//      ramp();                
//  }
//  old = toggle;
    if(toggle == 1)
    {
      coast(); 
    }



}

void PID_Function(IqSerial &serialPort, float motorRads, float *pid_p, float *pid_i, float *pid_d, float *prevPID, float *error, float *motortime, float *est_angle, float offset){
    // get the angle of the virtual reference
    desired_angle = hmodRadf(motorRads*(Time/1000));
    
    // get the angle of actual motor
    serialPort.get(mot.obs_angle_,angle);
    
    // find the difference between motor and "ref"
    // offset 0: motor is parallel to the virtual motor
    // offset PI/2: motor is 90 degrees off with respect to virtual motor
    diff = hmodRadf(desired_angle - angle + offset); 
    
    // new estimate angle
    // 1. propogate 
    //      angle += (measured rpm)*elapsedTime
    // 2. update  
    //      estimated diff = estimated angle - desired angle
    // update = ((diff-est_diff)->[-PI to PI])*((small_gain)->[0-1]))
    // est_diff += update
    // angle -= update 
    serialPort.get(mot.obs_velocity_, obs_vel);
    *est_angle += obs_vel*(*motortime);                  // estimate the angle of the motor using time since last PID call
    *est_angle = hmodRadf(*est_angle);                   // set est_angle between [-Pi, Pi]
  
    est_diff = desired_angle - (*est_angle) + offset;    // estimate the difference between the desired and estimated angle
    est_diff = hmodRadf(est_diff);                       // set est_diff between [-Pi, Pi]
  
    updateangle = hmodRadf(diff-est_diff)*0.1;
    est_diff += updateangle;
    *est_angle -= updateangle;
    *est_angle = hmodRadf(*est_angle);

    // Calculating P, I, D
    *pid_p = est_diff*kp;
    *pid_i += est_diff*(*motortime);
    //*pid_d = (motorRads - obs_vel)*kd; //not in use

    // Final PID 
    PID = *pid_p + ki*(*pid_i) + *pid_d;
    
    // Calculate the new velocity
    newvel = motorRads*GAIN + PID;  // gain value puts real motor speed closest to the virtual motor
    
    // Command new velocity to motor
    serialPort.set(prop.ctrl_velocity_,newvel);
    
    // Save current PID value for future PID
    *prevPID = PID;
    
    // Save current error for future PID
    *error = est_diff;
}

float hmodRadf(float h){
  float dh;
  int i;

  if (h>0){
    i = (int)(h/(2*PI)+0.5);
  }
  else{
    i = (int)(h/(2*PI)-0.5);
  }
  dh = h - PI*2*i;

  return dh;
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
