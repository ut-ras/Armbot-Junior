#include <Servo.h> 
#include <SPI.h>
#include <Wire.h>

 
Servo J1;  

// Control and feedback pins
int J1_PWM_Pin = 9;
int feedbackPin = A4;



// Calibration values
int minDegrees;
int maxDegrees;
int minFeedback;
int maxFeedback;
int tolerance = 1; // max feedback measurement error


int DFR_max = 615;
int DFR_min = 125;


void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
{
  // Move to the minimum position and record the feedback value
  servo.writeMicroseconds(inMicro(minPos));
  minDegrees = (minPos);
  delay(1000); // make sure it has time to get there and settle
  minFeedback = analogRead(analogPin);
  Serial.println("Actual Min: ");
  Serial.println(getPos(feedbackPin));

  
  // Move to the maximum position and record the feedback value
  servo.writeMicroseconds(inMicro(maxPos));
  maxDegrees = (maxPos);
  delay(1000); // make sure it has time to get there and settle
  maxFeedback = analogRead(analogPin);
    Serial.println("Actual Max: ");
    Serial.println(getPos(feedbackPin));

  Serial.println("Motor Calibrated");
  Serial.print("minDegrees: ");
  Serial.println(minDegrees);
  Serial.print("maxDegrees: ");
  Serial.println(maxDegrees);

  delay(3000);

}

void moveTo(int goal){
  if(goal > maxDegrees || goal < minDegrees){
    Serial.println("OUT OF BOUNDS");
    return;
  }
  Serial.println("MOVING TOWARDS: ");
  Serial.println(goal);

  goal = inMicro(goal);
  J1.writeMicroseconds(goal);
    delay(1000);
  Serial.println("Actual Angle: ");
  Serial.println(getPos(feedbackPin));
    delay(500);
}

int inDeg(int in){
  return abs(map(in, 495, 2505, 0, 270));
}
int inMicro(int in){
  return abs(map(in, 0, 270, 495, 2505));
}
int getPos(int analogPin){
  return abs(map(analogRead(analogPin), minFeedback, maxFeedback, minDegrees, maxDegrees));
}

void setup() 
{ 
  
  Serial.begin(9600);
  J1.attach(J1_PWM_Pin, DFR_min, DFR_max); 

  calibrate(J1, feedbackPin, 0, 180);  // calibrate for the mechanical joint range

} 



void loop()
{
  
moveTo(0);
delay(1000);
moveTo(90);
delay(1000);

  
  
}




