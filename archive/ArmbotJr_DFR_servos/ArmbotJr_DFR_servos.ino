#include <Servo.h> 
#include <SPI.h>
#include <Wire.h>

struct ServoConfig {
  char name[20];  // Added field for name, assuming a maximum length of 20 characters
  int PWM_pin;
  int Feedback_Pin;
  int minDegree;
  int maxDegree;
  int minFeedback;
  int maxFeedback;

  ServoConfig(const char* servo_name, int pwm, int feedback, int min_deg, int max_deg)
    : PWM_pin(pwm), Feedback_Pin(feedback), minDegree(min_deg), maxDegree(max_deg), minFeedback(0), maxFeedback(1023) {
      strncpy(name, servo_name, sizeof(name));
      name[sizeof(name) - 1] = '\0';  // Null-terminate in case the input string was too long
  }
};


ServoConfig Base_conf("Base", 9, A0, 0, 180); 
ServoConfig J1_conf("J1", 10, A1, 0, 180);
ServoConfig JX_conf("JX", 11, A2, 90, 180);

Servo Base;
Servo J1;
Servo JX;


int DFR_min = 2505;
int DFR_max = 495;


// This function calibrates a servo motor using a ServoConfig struct.
void calibrate(Servo &servo, ServoConfig config)
{
  // Attach the servo to its PWM pin
  servo.attach(config.PWM_pin, DFR_min, DFR_max);
  Serial.print("Calibrating Joint Servo: ");
  Serial.println(config.name);

  servo.writeMicroseconds(inMicro(config.minDegree));
  delay(1000);
  config.minFeedback = analogRead(config.Feedback_Pin);  // Update the config
  Serial.println("Actual Min: ");
  Serial.println(getPos(config));

  servo.writeMicroseconds(inMicro(config.maxDegree));
  delay(1000);
  config.maxFeedback = analogRead(config.Feedback_Pin);  // Update the config
  Serial.println("Actual Max: ");
  Serial.println(getPos(config));

  // Output calibration results
  Serial.println("Motor Calibrated!!!");
  Serial.print("Min Rototation: ");
  Serial.println(config.minDegree);
  Serial.print("Max Rotation: ");
  Serial.println(config.minDegree);

  delay(3000);  // Wait for 3 seconds before proceeding
}


void moveTo(Servo &servo, ServoConfig config, int goal){
  if(goal > config.maxDegree || goal < config.minDegree){
    Serial.println("OUT OF BOUNDS");
    return;
  }
  Serial.print("Moving Motor: ");
  Serial.print(config.name);
  Serial.print(" towards: ");
  Serial.println(goal);

  goal = inMicro(goal);
  J1.writeMicroseconds(goal);
    delay(1000);
  Serial.println("Actual Angle: ");
  Serial.println(getPos(config));
    delay(500);
}

int inDeg(int in){
  return abs(map(in, 495, 2505, 0, 270));
}
int inMicro(int in){
  return abs(map(in, 0, 270, 495, 2505));
}
int getPos(const ServoConfig& config) {
  return abs(map(analogRead(config.Feedback_Pin), config.minFeedback, config.maxFeedback, config.minDegree, config.maxDegree));
}


void setup() 
{ 
  
Serial.begin(9600);
calibrate(Base, Base_conf);
calibrate(J1, J1_conf);
calibrate(JX, JX_conf);

} 



void loop()
{
  
moveTo(Base, Base_conf, 0);
moveTo(Base, Base_conf, 180);

  
  
}




