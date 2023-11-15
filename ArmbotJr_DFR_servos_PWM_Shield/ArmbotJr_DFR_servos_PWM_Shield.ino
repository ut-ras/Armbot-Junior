#include "ArmbotJr_DFR_Functions.h"

const int mode = 0; 
// 0 = runs your own test code in loop()
// 1 = enable pose sequence "pseudo-multitasking" in loop(), still a WIP

// Create ServoConfig objects for each servo
ServoConfig Base("Base", 10, 0, 0, 195, -90, 90, 0);
ServoConfig J1("J1", 11, 4, 5, 180, -90, 90, 0);
ServoConfig J2("J2", 12, 2, 5, 270, -135, 135, 0);
ServoConfig J3("J3", 13, 15, 45, 270, -90, 135, 0);
ServoConfig J4("J4", 14, 13, 0, 270, -135, 135, 12);

ServoConfig allJoints[] = {Base, J1, J2, J3, J4};
const int numJoints = 5;


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
  Serial.println("Setup");

  // Initialize the Adafruit PWM servo driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Set PWM frequency to 60Hz
  //Locks all motors in current position (make sure in safe position before running the code)
  moveTo(Base, 0);  
  moveTo(J1, 0);
  moveTo(J2, 0);
  moveTo(J3, 0);
  moveTo(J4, 0); 
  // Calibrate all the servos
  //from the top down is usually safer
  calibrate(J4);
  calibrate(J3);
  calibrate(J2);
  calibrate(J1);
  calibrate(Base);



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
      moveTo(Base, nextPose.jointStates[0]);
      moveTo(J1, nextPose.jointStates[1]);
      moveTo(J2, nextPose.jointStates[2]);
      moveTo(J3, nextPose.jointStates[3]);
      moveTo(J4, nextPose.jointStates[4]);
      // Update the current pose index for the next iteration
      currentPoseIndex = (currentPoseIndex + 1) % numPoses;
    }
    // Task 2: Continuously print the position at 30 times a second
    if (currentMillis - previousMillisPrint >= intervalPrint) {
      // Save the last time this task was executed
      previousMillisPrint = currentMillis;
    printAllJointPos(allJoints, numJoints);
    }
  }

  else if(mode==0){
   // Add your own test code here

  //for example:
  moveTo(J4, -90);
  BinaryClaw(closed);
  delay(3000);
  moveTo(J4, 90);
  BinaryClaw(open);
  delay(3000);
  }
}