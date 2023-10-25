// Include necessary libraries
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

int mode = 1; //0 = calibrations only, 1 = enable pose sequence (void loop)

unsigned long previousMillisPose = 0;  // will store last time a pose was executed
unsigned long previousMillisPrint = 0;  // will store last time position was printed
const long intervalPose = 1500;  // 1.5 seconds per pose
const long intervalPrint = 100;  // fast printing 
int currentPoseIndex = 0;


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Constants for control pulse lengths
// from 500us = 0deg per the datasheet;
//(500us * 60Hz)*4096 = 122.9
const int DFR_min = 125; 

 // from 2500us = 270deg per the datasheet
 //(2500us * 60Hz)*4096 = 614.4
const int DFR_max = 615;
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

// Create ServoConfig objects for each servo
ServoConfig Base_conf("Base", 0, A0, 0, 195, -90, 90, 0);
ServoConfig J1_conf("J1", 1, A1, 5, 180, -90, 90, 0);
ServoConfig J2_conf("J2", 2, A2, 5, 270, -135, 135, 0);
ServoConfig allJoints[] = {Base_conf, J1_conf, J2_conf};  // Add more joints here when you expand your arm
const int numJoints = sizeof(allJoints) / sizeof(ServoConfig);


const int numPoses = 10;  // Number of poses in the sequence
Pose poseSequence[numPoses] = {
  Pose(0, 0, 0, 0, 0),
  Pose(10, -20, 60, 0, 0),
  Pose(-30, 40, -120, 0, 0),
  Pose(50, -10, 100, 0, 0),
  Pose(-20, 30, -90, 0, 0),
  Pose(40, -50, 135, 0, 0),
  Pose(-60, 20, -80, 0, 0),
  Pose(30, -40, 110, 0, 0),
  Pose(-10, 50, -130, 0, 0),
  Pose(20, -30, 85, 0, 0)
};



// Calibrate a servo
void calibrate(ServoConfig &config) {
  Serial.print("Calibrating servo: ");
  Serial.println(config.name);

  // Calculate the pulse length for the min and max position
  int pulseLen_min = map(config.minDegree, 0, 270, DFR_min, DFR_max);
  int pulseLen_max = map(config.maxDegree, 0, 270, DFR_min, DFR_max);

  pwm.setPWM(config.PWM_Channel, 0, pulseLen_min);
  delay(2000);
  // Read min feedback and store it in config object
  config.minFeedback = analogRead(config.Feedback_Pin);
  Serial.println(config.minFeedback);
  Serial.print("Actual Min: ");
  Serial.println(getJointPos(config));

  pwm.setPWM(config.PWM_Channel, 0, pulseLen_max);
  delay(2000);
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
  calibrate(J2_conf);

  // Setup the pose sequence
 // setupPoseSequence();
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


// Loop function for continuously running the program
void loop() {
          if(mode==1){
          unsigned long currentMillis = millis();

          // Task 1: Cycle through poses
          if (currentMillis - previousMillisPose >= intervalPose) {
            // Save the last time a pose was executed
            previousMillisPose = currentMillis;

            // Get the next pose from the sequence
            Pose nextPose = poseSequence[currentPoseIndex];

            // Move to the next pose
            moveTo(Base_conf, nextPose.jointStates[0]);
            moveTo(J1_conf, nextPose.jointStates[1]);
            moveTo(J2_conf, nextPose.jointStates[2]);
            // Add more joints here when you expand your arm

            // Update the current pose index for the next iteration
            currentPoseIndex = (currentPoseIndex + 1) % numPoses;
          }

          // Task 2: Continuously print the position at 30 times a second
          if (currentMillis - previousMillisPrint >= intervalPrint) {
            // Save the last time this task was executed
            previousMillisPrint = currentMillis;

            // Your code to print the position
          printAllJointPos(allJoints, numJoints);
            // Add more joints here when you expand your arm
          }
        }

  else if(mode==0){
    moveTo(J2_conf, -135);
    delay(3000);
    moveTo(J2_conf, +135);
    delay(3000);
  }
}
