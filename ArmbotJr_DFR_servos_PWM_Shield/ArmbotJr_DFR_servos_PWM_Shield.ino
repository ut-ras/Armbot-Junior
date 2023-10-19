#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// Initialize the Adafruit PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct ServoConfig {
  char name[20];
  int PWM_pin;
  int Feedback_Pin;
  int minDegree;
  int maxDegree;
  int minFeedback;
  int maxFeedback;

  ServoConfig(const char* servo_name, int channel, int feedback, int min_deg, int max_deg)
    : PWM_pin(channel), Feedback_Pin(feedback), minDegree(min_deg), maxDegree(max_deg), minFeedback(0), maxFeedback(1023) {
      strncpy(name, servo_name, sizeof(name));
      name[sizeof(name) - 1] = '\0';
  }
};

ServoConfig Base_conf("Base", 0, A0, 0, 180);
ServoConfig J1_conf("J1", 1, A1, 0, 180);
ServoConfig JX_conf("JX", 2, A2, 90, 180);

int DFR_min = 2505;
int DFR_max = 495;

void calibrate(ServoConfig &config) {
  Serial.print("Calibrating servo: ");
  Serial.println(config.name);

  // Note: Using Adafruit library, you generally don't 'attach' like in the standard Servo library
  // Instead, you will directly write to the servo with pwm.setPWM()

  // Read min feedback
  config.minFeedback = analogRead(config.Feedback_Pin);
  Serial.print("Actual Min: ");
  Serial.println(getPos(config));

  // Read max feedback
  config.maxFeedback = analogRead(config.Feedback_Pin);
  Serial.print("Actual Max: ");
  Serial.println(getPos(config));

  // Your calibration logic here

  Serial.println("Motor Calibrated");
}

int getPos(const ServoConfig &config) {
  return abs(map(analogRead(config.Feedback_Pin), config.minFeedback, config.maxFeedback, config.minDegree, config.maxDegree));
}

void moveTo(ServoConfig &config, int goal) {
  if (goal > config.maxDegree || goal < config.minDegree) {
    Serial.println("Out of bounds");
    return;
  }
  
  Serial.print("Moving ");
  Serial.print(config.name);
  Serial.print(" to ");
  Serial.println(goal);

  // Calculate the pulse length
  int pulseLen = map(goal, config.minDegree, config.maxDegree, DFR_min, DFR_max);

  // Move the servo
  pwm.setPWM(config.PWM_pin, 0, pulseLen);

  // Your other code here
}

void setup() {
  Serial.begin(9600);

  // Initialize the Adafruit PWM servo driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // Calibrate the servos
  calibrate(Base_conf);
  calibrate(J1_conf);
  calibrate(JX_conf);
}

void loop() {
  moveTo(Base_conf, 0);
  moveTo(Base_conf, 180);
}
