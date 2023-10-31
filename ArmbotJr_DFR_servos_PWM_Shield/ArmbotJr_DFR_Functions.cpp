#include "ArmbotJr_DFR_Functions.h"

unsigned long previousMillisPose = 0;  // will store last time a pose was executed
unsigned long previousMillisPrint = 0;  // will store last time position was printed
long intervalPose = 1500;  // 1.5 seconds per pose
long intervalPrint = 100;  // fast printing 
int currentPoseIndex = 0;
// Constants for control pulse lengths
// from 500us = 0deg per the datasheet;
//(500us * 60Hz)*4096 = 122.9
int DFR_min = 125; 
 // from 2500us = 270deg per the datasheet
 //(2500us * 60Hz)*4096 = 614.4
int DFR_max = 615;
//sets up the pwm shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


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
  Serial.println(getJointPos(config));

  pwm.setPWM(config.PWM_Channel, 0, pulseLen_max);
  delay(3000);
  // Read max feedback and store it in config object
  config.maxFeedback = analogRead(config.Feedback_Pin);
  Serial.println(config.maxFeedback);
  Serial.print("Actual Max: ");
  Serial.println(getJointPos(config));

  // Move the servo to default position
  bool res = moveTo(config, config.defaultPos);
  if (!res) {
    Serial.println("Failed to move to default position");
    return;
  }
  Serial.println("Motor Calibrated");
  
  return;
}

// Translate servo angle to joint angle
int servoToJoint(int servo_angle, const ServoConfig &config) {
  return map(servo_angle, config.minDegree, config.maxDegree, config.jointMinDegree, config.jointMaxDegree);
}

// Translate joint angle to servo angle
int jointToServo(int joint_angle, const ServoConfig &config) {
  return map(joint_angle, config.jointMinDegree, config.jointMaxDegree, config.minDegree, config.maxDegree);
}

// Get current position of servo in joint space
int getJointPos(const ServoConfig &config) {
  int servo_angle = map(analogRead(config.Feedback_Pin), config.minFeedback, config.maxFeedback, config.minDegree, config.maxDegree);
  return servoToJoint(servo_angle, config);
}
// Move servo to a specific position in joint space
bool moveTo(ServoConfig &config, int joint_goal) {
  int servo_goal = jointToServo(joint_goal, config);
  if (servo_goal > config.maxDegree || servo_goal < config.minDegree) {
    Serial.println("Out of bounds!!!");
    return false;
  }  
//  Serial.print("Moving Joint "); Serial.print(config.name); Serial.print(" to "); Serial.println(joint_goal);
  // Calculate the pulse length for the given position
  int pulseLen = map(servo_goal, 0, 270, DFR_min, DFR_max);
  // Move the servo to desired position
  pwm.setPWM(config.PWM_Channel, 0, pulseLen);
  //delay (1500); //wait for joint to move
  // Check if the servo moved to desired position
int diff = (getJointPos(config) - joint_goal);
/*if (abs(diff) > 1)
  {
    Serial.println("Failed to move to desired position, fixing...");
  moveTo(config, (goal + diff));
  }*/
  //if we made it and alls good:
  return true;
}
void printPos(const ServoConfig &config) {
  Serial.print(config.name);
  Serial.print(": ");
  Serial.print(getJointPos(config));  // Assuming getPos() returns the joint position
  Serial.print(" deg");
}
void printAllJointPos(ServoConfig configs[], int numJoints) {

  //Serial.println("Joint Positions:");
  for (int i = 0; i < numJoints; ++i) {
    Serial.print(configs[i].name);
    Serial.print(":");
    // Add spaces for alignment
    for (int j = strlen(configs[i].name); j < 10; ++j) {
      Serial.print(" ");
    }
    Serial.println(getJointPos(configs[i]));
  }
  Serial.println();  // Add an extra line for separation
}



