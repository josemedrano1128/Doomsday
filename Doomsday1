/*
  Analog input, analog output, serial output

 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
 
 #include <Servo.h> 

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInX = A0;  // Analog input pin that the potentiometer is attached to
const int analogInY = A1;
const int analogInW = A2;
const int firePin = 7;
const int laserPin = 8;
//const int analogOutPin = 9; // Analog output pin that the LED is attached to

int XValue = 0;        // value read from the pot
int YValue = 0;
int WValue = 0;
int fire = 0;
int laser = 0;

Servo leftservo;
Servo rightservo;
Servo baseservo;
Servo fireservo;

//int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  leftservo.attach(3);  
  rightservo.attach(5);
  //baseservo.attach(6);
  //fireservo.attach(9);
  
}

void loop() {
  // read the analog in value:
  XValue = analogRead(analogInX);
  YValue = analogRead(analogInY);
  WValue = analogRead(analogInW);
  fire = !digitalRead(firePin);
  laser = !digitalRead(laserPin);
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);

  // print the results to the serial monitor:
  Serial.print("X = " );
  Serial.print(XValue);
  Serial.print("\t Y = " );
  Serial.print(YValue);
  Serial.print("\t Wheel = " );
  Serial.print(WValue);
  Serial.print("\t Laser = ");
  Serial.print(laser);
  Serial.print("\t Fire = ");
  Serial.println(fire);
  //Xvalue = 
  YValue = map(YValue, 567, 1023, 0, 180);
  //Wvalue = 
  leftservo.write(YValue);
  rightservo.write(180-YValue);
  
  //Serial.print("\t output = ");
  //Serial.println(outputValue);

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(100);
}
