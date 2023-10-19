// Include necessary libraries
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// Create a structure to hold Servo configuration
struct ServoConfig {
  char name[20];  // Holds servo name
  int PWM_Channel;    // Holds PWM pin number connected to the servo
  int Feedback_Pin;  // Holds Feedback pin number receiving feedback from servo
  int minDegree;  // Holds the minimum limit of the servo
  int maxDegree;  // Holds maximum limit of the servo
  int minFeedback;  // Holds the minimum feedback from the servo
  int maxFeedback;  // Holds the maximum feedback from the servo

  // Constructor for struct ServoConfig
  ServoConfig(const char* servo_name, int channel, int feedback, int min_deg, int max_deg)
    : PWM_Channel(channel), Feedback_Pin(feedback), minDegree(min_deg), maxDegree(max_deg), minFeedback(0), maxFeedback(1023) {
    strncpy(name, servo_name, sizeof(name));
    name[sizeof(name) - 1] = '\0';
  }
};

// Create ServoConfig objects for each servo
ServoConfig Base_conf("Base", 0, A0, 0, 180);
ServoConfig J1_conf("J1", 1, A1, 0, 180);
ServoConfig JX_conf("JX", 2, A2, 90, 180);

// Define constants for PWM pulse length, 
// These are given by the manufacturer of the DFRobot Servo's
int DFR_min = 2505;
int DFR_max = 495;

// Calibrate a servo
void calibrate(ServoConfig &config) {
  Serial.print("Calibrating servo: ");
  Serial.println(config.name);

  // Read min feedback and store it in config object
  config.minFeedback = analogRead(config.Feedback_Pin);
  Serial.print("Actual Min: ");
  Serial.println(getPos(config));

  // Read max feedback and store it in config object
  config.maxFeedback = analogRead(config.Feedback_Pin);
  Serial.print("Actual Max: ");
  Serial.println(getPos(config));

  // Calibration logic could go here if needed

  Serial.println("Motor Calibrated");
}

// Get current position of servo
int getPos(const ServoConfig &config) {
  return abs(map(analogRead(config.Feedback_Pin), config.minFeedback, config.maxFeedback, config.minDegree, config.maxDegree));
}

// Move servo to a specific position
void moveTo(ServoConfig &config, int goal) {
  // Check if the goal is within servo's limit
  if (goal > config.maxDegree || goal < config.minDegree) {
    Serial.println("Out of bounds");
    return;
  }
  
  Serial.print("Moving ");
  Serial.print(config.name);
  Serial.print(" to ");
  Serial.println(goal);

  // Calculate the pulse length for the given position
  int pulseLen = map(goal, config.minDegree, config.maxDegree, DFR_min, DFR_max);

  // Move the servo to desired position
  pwm.setPWM(config.PWM_pin, 0, pulseLen);
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
  calibrate(JX_conf);
}

// Loop function for continuously running the program
void loop() {
  // Move the Base servo back and forth between 0 and 180 degrees
  moveTo(Base_conf, 0);
  moveTo(Base_conf, 180);
}
