// Include necessary libraries
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Constants for control pulse lengths
// from 500us = 0deg per the datasheet;
//(500us * 60Hz)*4096 = 122.9
const int DFR_min = 125; 

 // from 2500us = 270deg per the datasheet
 //(2500us * 60Hz)*4096 = 614.4
const int DFR_max = 615;

// Create a structure to hold Servo configuration
struct ServoConfig {
  char name[20];  // Holds servo name
  int PWM_Channel;    // Holds PWM pin number connected to the servo
  int Feedback_Pin;  // Holds Feedback pin number receiving feedback from servo
  int minDegree;  // Holds the minimum limit of the servo
  int maxDegree;  // Holds maximum limit of the servo
  int minFeedback;  // Holds the minimum feedback from the servo
  int maxFeedback;  // Holds the maximum feedback from the servo
  int defaultPos;  // Holds the default (safe) position of the joint

  // Constructor for struct ServoConfig
  ServoConfig(const char* servo_name, int channel, int feedback, int min_deg, int max_deg, int default_pos)
    : PWM_Channel(channel), Feedback_Pin(feedback), minDegree(min_deg), maxDegree(max_deg), minFeedback(0), maxFeedback(1023), defaultPos(default_pos) {
    strncpy(name, servo_name, sizeof(name));
    name[sizeof(name) - 1] = '\0';
  }
};

// Create ServoConfig objects for each servo
ServoConfig Base_conf("Base", 0, A0, 0, 180, 90);
ServoConfig J1_conf("J1", 1, A1, 5, 180, 90);
ServoConfig J2_conf("J2", 2, A2, 5, 267, 129);



// Calibrate a servo
void calibrate(ServoConfig &config) {
  Serial.print("Calibrating servo: ");
  Serial.println(config.name);

  // Calculate the pulse length for the min and max position
  int pulseLen_min = map(config.minDegree, 0, 270, DFR_min, DFR_max);
  int pulseLen_max = map(config.maxDegree, 0, 270, DFR_min, DFR_max);

  pwm.setPWM(config.PWM_Channel, 0, pulseLen_min);
  delay(3000);
  // Read min feedback and store it in config object
  config.minFeedback = analogRead(config.Feedback_Pin);
  Serial.println(config.minFeedback);
  Serial.print("Actual Min: ");
  Serial.println(getPos(config));

  pwm.setPWM(config.PWM_Channel, 0, pulseLen_max);
  delay(3000);
  // Read max feedback and store it in config object
  config.maxFeedback = analogRead(config.Feedback_Pin);
  Serial.println(config.maxFeedback);
  Serial.print("Actual Max: ");
  Serial.println(getPos(config));

  // Move the servo to default position
  bool res = moveTo(config, config.defaultPos);
  if (!res) {
    Serial.println("Failed to move to default position");
    return;
  }
  Serial.println("Motor Calibrated");
  delay((3000));
  return;
}

// Get current position of servo
int getPos(const ServoConfig &config) {
  return abs(map(analogRead(config.Feedback_Pin), config.minFeedback, config.maxFeedback, config.minDegree, config.maxDegree));
}

// Move servo to a specific position
bool moveTo(ServoConfig &config, int goal) {
  // Check if the goal is within servo's limit
  if (goal > config.maxDegree || goal < config.minDegree) {
    Serial.println("Out of bounds!!!");
    return false;
  }
  
  Serial.print("Moving ");
  Serial.print(config.name);
  Serial.print(" to ");
  Serial.println(goal);
  // Calculate the pulse length for the given position
  int pulseLen = map(goal, 0, 270, DFR_min, DFR_max);
  // Move the servo to desired position
  pwm.setPWM(config.PWM_Channel, 0, pulseLen);
  delay (1500); //wait for joint to move
  // Check if the servo moved to desired position
 int diff = (getPos(config) - goal);
if (abs(diff) > 1)
  {
    Serial.println("Failed to move to desired position, fixing...");
  moveTo(config, (goal + diff));
  }
  //if we made it and alls good:
  return true;
}

// Setup function for initializing the program
void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize the Adafruit PWM servo driver

  pwm.begin();
  pwm.setPWMFreq(60);  // Set PWM frequency to 60Hz

  // Calibrate all the servos
  calibrate(Base_conf);
  calibrate(J1_conf);
  //calibrate(JX_conf);
}

void printPos(const ServoConfig &config) {
  Serial.print(config.name);
  Serial.print(" Position: ");
  Serial.println(getPos(config));
}
// Loop function for continuously running the program
void loop() {
  // Move the Base servo back and forth between 0 and 180 degrees

}
