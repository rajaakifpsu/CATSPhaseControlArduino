// Upload 900 (intialize)
// Turn on Power supply
//  Upload run code (ex.1500)



#include <Servo.h>

Servo ESC;     // create servo object to control the ESC
Servo ESC1;

int value;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  //ESC.attach(9,900,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC.attach(9);
  ESC1.attach(5);
  ESC.writeMicroseconds(900);
  ESC1.writeMicroseconds(900);
  Serial.begin(9600); 
   
}

void loop() {
  //if(++value > 100)
  //{
  //  value=0;
  //}
  
  value = 1400;
 
  //value = Serial.parseInt();
  
  //Serial.println(value);
  //value = map(value, 0, 100, 1000, 3000);
  
  ESC.write(value);    // Send the signal to the ESC
  ESC1.write(value);
}

// Hello world!