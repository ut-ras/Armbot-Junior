#include "ArmbotJr_DFR_Functions.h"

const int mode = 1; 
// 0 = calibrations and then add your own test code in the loop
// 1 = enable pose sequence (void loop)

// Create ServoConfig objects for each servo
ServoConfig Base_conf("Base", 0, A0, 0, 195, -90, 90, 0);
ServoConfig J1_conf("J1", 1, A1, 5, 180, -90, 90, 0);
ServoConfig J2_conf("J2", 2, A2, 5, 270, -135, 135, 0);
ServoConfig J3_conf("J3", 3, A3, 0, 270, -135, 135, 0);
ServoConfig J4_conf("J4", 4, A4, 0, 270, -135, 135, 12);

ServoConfig allJoints[] = {Base_conf, J1_conf, J2_conf, J3_conf, J4_conf};  // Add more joints here after
const int numJoints = sizeof(allJoints) / sizeof(ServoConfig);
const int numPoses = 1;  // Number of poses in the sequence
Pose poseSequence[numPoses] = {
  Pose(0, -45, 90, -90, 90)
};


// Setup function for initializing the program
void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize the Adafruit PWM servo driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Set PWM frequency to 60Hz
 Serial.println("Setup");
  // Calibrate all the servos
  moveTo(Base_conf, 0);  // Move to 0 degrees
  moveTo(J1_conf, 0);
  moveTo(J2_conf, 0);
  moveTo(J3_conf, 0);
  moveTo(J4_conf, 0);

  calibrate(J4_conf);
  calibrate(J3_conf);
  calibrate(J2_conf);
  calibrate(J1_conf);
  calibrate(Base_conf);

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
            moveTo(J3_conf, nextPose.jointStates[3]);
            moveTo(J4_conf, nextPose.jointStates[4]);


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
   
  /* for (size_t i = -135; i <= 0; i++)
  {
    moveTo(J4_conf, i);
    Serial.println("ITS AT ");
    Serial.println(i);
    delay(1500);
  }
  Serial.println("ITS AT 0");
  delay(5000);
  
  for (size_t i = 0; i <= 135; i++)
  {
    moveTo(J4_conf, i);
    Serial.println("ITS AT ");
    Serial.println(i);
    delay(1500);
  } */
  moveTo(J4_conf, -135);
  delay(5000);
}
}