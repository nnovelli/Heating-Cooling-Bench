#include "DualG2HighPowerMotorShield.h"

// Uncomment the version corresponding with the version of your shield.
//DualG2HighPowerMotorShield24v14 md;
// DualG2HighPowerMotorShield18v18 md;
 DualG2HighPowerMotorShield24v18 md;
// DualG2HighPowerMotorShield18v22 md;

const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
int buttonState = 0;         // variable for reading the pushbutton status
int potpin = A5;
int spd;
int redPin = 7;
int greenPin = 6;
int bluePin = 5;
int potMin = 0;
int potMax = 675;
int avg;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
  Serial.begin(115200);
  Serial.println("Hot/Cold Seat Test");
  md.init();
  md.calibrateCurrentOffsets();

  delay(10);

  // Uncomment to flip a motor's direction:
  md.flipM1(true);
  md.flipM2(true);
}

void loop() {
  md.enableDrivers();
  buttonState = digitalRead(buttonPin);
  int val = analogRead(potpin);
  //potspeed(val);
  avg = map(val, potMin, potMax, 0, 4);
  spd = map(avg, 0, 4, -400, 400);
  //Serial.print(spd);
  //md.setM1Speed(spd);
  md.setM2Speed(spd);
  //stopIfFault(); 
  //int range = map(val, potMin, potMax, 0, 4);
    switch (avg) {
    case 0: 
      setColor(0, 0, 200);
      break;
    case 1:  
      setColor(0, 0, 90); 
      break;
    case 2:
      setColor(0, 0, 0);  
      break;
    case 3: 
      setColor(90, 0, 0); 
      break;
   case 4: 
      setColor(200, 0, 0);
      break;      
  }
  }


void stopIfFault()
{
  if (md.getM1Fault())
  {
    md.disableDrivers();
  delay(1);
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
    md.disableDrivers();
  delay(1);
    Serial.println("M2 fault");
    while (1);
  }
}

void potspeed(int val){
  spd = map(val, potMin, potMax, -400, 400);
  return spd;
  }

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
