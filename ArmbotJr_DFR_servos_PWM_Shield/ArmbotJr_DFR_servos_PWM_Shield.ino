#include "ArmbotJr_DFR_Functions.h"

const int mode = 0; 
// 0 = calibrations and then add your own test code in the loop
// 1 = enable pose sequence (void loop)

// Create ServoConfig objects for each servo
ServoConfig Base_conf("Base", 0, A0, 0, 195, -90, 90, 0);
ServoConfig J1_conf("J1", 1, A1, 5, 180, -90, 90, 0);
ServoConfig J2_conf("J2", 2, A2, 5, 270, -135, 135, 0);
ServoConfig J3_conf("J3", 3, A3, 45, 270, -90, 135, 0);
ServoConfig J4_conf("J4", 4, A4, 0, 270, -135, 135, 12);

ServoConfig allJoints[] = {Base_conf, J1_conf, J2_conf, J3_conf, J4_conf};  // Add more joints here after
const int numJoints = sizeof(allJoints) / sizeof(ServoConfig);


//can be any number of poses, just make sure to change numPoses to the correct number
const int numPoses = 2;  // Number of poses in the sequence
Pose poseSequence[numPoses] = { // Sequence of user-defined poses to test in the loop
  Pose(0, -45, 90, -90, 90, closed),
  Pose(0, -45, 90, -90, 90, open)
};


// Setup function for initializing the program
void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize the Adafruit PWM servo driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Set PWM frequency to 60Hz
 Serial.println("Setup");
  //Lock all motors in current position (make sure in safe position before running the code)
  moveTo(Base_conf, 0);  
  moveTo(J1_conf, 0);
  moveTo(J2_conf, 0);
  moveTo(J3_conf, 0);
  moveTo(J4_conf, 0); 
  // Calibrate all the servos
  //from the top down is usually safer
  calibrate(J4_conf);
  calibrate(J3_conf);
  calibrate(J2_conf);
  calibrate(J1_conf);
  calibrate(Base_conf);

}

// Loop function for continuously running the program
void loop() {
  if(mode==1){ //pseudo-multitasking mode, runs the pose sequence and prints the position at kind of the same time
    unsigned long currentMillis = millis();
    // Task 1: Cycle through poses
    if (currentMillis - previousMillisPose >= intervalPose) {
      // Save the last time a pose was executed
      previousMillisPose = currentMillis;
      // Get the next pose from the sequence
      Pose nextPose = poseSequence[currentPoseIndex];
      // Move to the next pose
      BinaryClaw(nextPose.jointStates[5]);
      moveTo(Base_conf, nextPose.jointStates[0]);
      moveTo(J1_conf, nextPose.jointStates[1]);
      moveTo(J2_conf, nextPose.jointStates[2]);
      moveTo(J3_conf, nextPose.jointStates[3]);
      moveTo(J4_conf, nextPose.jointStates[4]);
      // Update the current pose index for the next iteration
      currentPoseIndex = (currentPoseIndex + 1) % numPoses;
    }
    // Task 2: Continuously print the position at 30 times a second
    if (currentMillis - previousMillisPrint >= intervalPrint) {
      // Save the last time this task was executed
      previousMillisPrint = currentMillis;
    printAllJointPos(allJoints, numJoints);
      // Add more joints here when you expand your arm
    }
  }

  else if(mode==0){
   // Add your own test code here
  moveTo(J4_conf, -90);
  BinaryClaw(closed);
  delay(3000);
  moveTo(J4_conf, 90);
  BinaryClaw(open);
  delay(3000);
  }
}