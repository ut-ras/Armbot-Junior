#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#ifndef ARMBOTJR_DFR_FUNCTIONS_H
#define ARMBOTJR_DFR_FUNCTIONS_H
extern unsigned long previousMillisPose;
extern unsigned long previousMillisPrint;
extern long intervalPose;
extern long intervalPrint;
extern int currentPoseIndex;
extern int DFR_min; 
extern int DFR_max;
extern Adafruit_PWMServoDriver pwm;

// Create a structure to hold Servo configuration
struct ServoConfig {
  char name[20];  // Holds servo name
  int PWM_Channel;    // Holds PWM pin number connected to the servo
  int Feedback_Pin;  // Holds Feedback pin number receiving feedback from servo
  int minDegree;  // Holds the minimum limit of the servo
  int maxDegree;  // Holds maximum limit of the servo

  int jointMinDegree;  // Holds the minimum limit of the joint in joint space
  int jointMaxDegree;  // Holds the maximum limit of the joint in joint space

  int minFeedback;  // Holds the minimum feedback from the servo
  int maxFeedback;  // Holds the maximum feedback from the servo
  int defaultPos;  // Holds the default (safe) position of the joint

  // Constructor for struct ServoConfig
  ServoConfig(const char* servo_name, int channel, int feedback, int min_deg, int max_deg, int joint_min_deg, int joint_max_deg, int default_pos)
    : PWM_Channel(channel), Feedback_Pin(feedback), minDegree(min_deg), maxDegree(max_deg), jointMinDegree(joint_min_deg), jointMaxDegree(joint_max_deg), minFeedback(0), maxFeedback(1023), defaultPos(default_pos) {
    strncpy(name, servo_name, sizeof(name));
    name[sizeof(name) - 1] = '\0';
  }

};

//Struct defintion for holding poses
struct Pose {
  int jointStates[5];
  
  Pose() {
    for (int i = 0; i < 5; ++i) {
      jointStates[i] = 0;
    }
  }
  
  Pose(int base, int j1, int j2, int j3, int j4) {
    jointStates[0] = base;
    jointStates[1] = j1;
    jointStates[2] = j2;
    jointStates[3] = j3;
    jointStates[4] = j4;
  }
};
extern Pose poseSequence[];



// Calibrate a servo
void calibrate(ServoConfig &config);

// Translate servo angle to joint angle
int servoToJoint(int servo_angle, const ServoConfig &config);

// Translate joint angle to servo angle
int jointToServo(int joint_angle, const ServoConfig &config);

// Get current position of servo in joint space
int getJointPos(const ServoConfig &config);

// Move servo to a specific position in joint space
bool moveTo(ServoConfig &config, int joint_goal);

// Print out positions of selected servo
void printPos(const ServoConfig &config);
// Print out positions of all JOINTS
void printAllJointPos(ServoConfig configs[], int numJoints);
#endif