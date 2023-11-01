# Armbot-Junior 

This repository contains all the necessary documentation and code base for the Armbot-Junior Demobots project.

## PLEASE NOTE THAT THE README IS STILL A WORK IN PROGRESS, DESCRIPTIONS OF CODE AND FUNCTIONS ARE NOT COMPLETE AND MAY BE INACCURATE.

## Introduction

Armbot-Junior is a miniature 6DOF (Degrees of Freedom) robotic arm project. It is primarily designed to serve as a test bed for the main Armbot project and as an educational tool. This project uses DFRobot's Servos with an internal analog feedback potentiometer, enabling the user to get real-time position data of the robot arm.

## Software Guide

Download the Arduino IDE either standalone or as a VSCode extension (or use platformIO if you're crazy). Clone the repository and open the project in the IDE. Be sure to install the Adafruit PWM, (`Adafruit_PWMServoDriver.h`), library from the Arduino Library Manager or elsewhere

1. [`ArmbotJr_DFR_servos_PWM_Shield.ino`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_servos_PWM_Shield.ino): The current working file. It's similar to `ArmbotJr_DFR_servos` but is designed for use with an Adafruit 16-Channel 12-bit PWM/Servo Driver Shield or a similar I2C device.

2. [`ArmbotJr_DFR_Functions.h`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_Functions.h): Header file, includes the needed libraries, contains the `ServoConfig` struct, the `Pose` struct, global variables and function prototypes for the `ArmbotJr_DFR_servos_PWM_Shield.ino` program.

3. [`ArmbotJr_DFR_Functions.cpp`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_Functions.cpp): Source file, contains the function  definitions for the `ArmbotJr_DFR_servos_PWM_Shield.ino` program.


This Arduino Sketch was originally in a single file, but was split into a "main" `.ino` file, a header file, and a source file to make it easier to read and understand.



### [`ArmbotJr_DFR_servos_PWM_Shield.ino`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_servos_PWM_Shield.ino) Explained

The `ArmbotJr_DFR_servos_PWM_Shield` program works as follows:

A struct named `ServoConfig` is defined to hold the configuration details of each servo such as name, PWM pin number, feedback pin number, a minimum and maximum degree value, and a minimum and maximum feedback value (for use in calibration). A constructor is also defined which accepts these parameters to create an object of the struct. 

###### (*A constructor is a special kind of function that gets called automatically when an object of the struct is created. Its main purpose is to assign initial values to the data members of the new object. It helps ensure that objects are valid as soon as they're created and simplifies code since initialization details are kept within the struct. Honestly this shouldnt really be done with a struct, I just kind of suck using Classes properly, add that to the TODO list lol*) 


```cpp
struct ServoConfig {
  char name[20];
  int PWM_Channel;
  int Feedback_Pin;
  int minDegree;
  int maxDegree;
  int minFeedback;
  int maxFeedback;

    // Constructor for struct ServoConfig
  ServoConfig(const char* servo_name, int channel, int feedback, int min_deg, int max_deg)
    : PWM_Channel(channel), Feedback_Pin(feedback), minDegree(min_deg), maxDegree(max_deg), minFeedback(0), maxFeedback(1023) {
    strncpy(name, servo_name, sizeof(name));
    name[sizeof(name) - 1] = '\0';
  }
};
```


As of writing, only two servos are initialized using the `ServoConfig struct` - "Base" and "J1".

***Please note, "JX" is just a placeholder for additional servos that are yet to be added.***

The `minDegree` and `maxDegree` values are the actual movment limits of the arm joint associated with each servo motor. For example, the Base joint moves from 0 to 180 degrees, so at the top of the code (or in the `setup()` function), it may be configured with:

```
ServoConfig Base("Base", 0, A0, 0, 180); 
```
Where `"Base"` is the joint's name, `0` is the PWM Channel, corresponding to where the servo cable is plugged into on the PWM Shield, `A0` is the pin connected to servo's white feedback signal wire, and `0, 180` are the minimum and maximum rotation/angle the base joint can physically move.

You can configure the servo like this at the start of the code, or within the `setup()` function.

After configuring, you'll now be able to "pass in" `Base` to functions like `calibrate()`, `moveTo()`, and `getPos()`.

### The `setup()` function

In the `setup()` function, serial communication is initiated to allow for monitoring of the program.

```cpp
Serial.begin(9600);
```

Here we also initialize the Adafruit PWM Servo Driver. Don't forget to include this part in your code.
```
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
```
Calibration for each of the servos is performed here as well.

```cpp
calibrate(Base, Base_conf);
calibrate(J1, J1_conf);
calibrate(JX, JX_conf);
```


###  Here's how `calibrate()` function works:
The `calibrate(ServoConfig &config)` function is used to calibrate a servo. This function helps to adjust the servo to get accurate movement.

The function takes a `ServoConfig` object as an argument which contains the configuration details of the servo which needs to be calibrated.

Inside this function, it first prints the name of the servo being calibrated to the serial monitor.

Then it reads the minimum and maximum feedback from the servo. The `analogRead()` function is used on the `Feedback_Pin` of the `ServoConfig` object to get these values. These values are then stored inside `minFeedback` and `maxFeedback` attributes of the `ServoConfig` object.

These minimum and maximum feedback values are printed on the serial monitor for the user's reference.

For example,Let's say you want to calibrate the `Base_conf` servo:

```cpp
calibrate(Base_conf);
```
In this case, the `Base_conf object` is passed as an argument to the `calibrate()` function. The function will then calibrate the base servo according to its defined `ServoConfig` settings and provide you the calibration results on the serial monitor.

Please note that the calibration should be performed during the setup phase of the Arduino program in the `setup()` function.

### The `loop()` Function

The `loop()` function is where you put your `moveTo()` function calls. For example:

```cpp
void loop() {
  moveTo(Base, Base_conf, 0);
  moveTo(Base, Base_conf, 180);
}
```

In this example, the `Base` servo moves between 0 and 180 degrees.


### The `moveTo()` Function Explained

`moveTo(ServoConfig &config, int goal)` function is used to move a servo to a target position. It checks if the target position is within the servo's valid degree range. If yes, it maps the degree to a pulse length and moves the servo to the target position.

The function starts by ensuring the angle you want to move the servo towards, the `goal`, is within the range of the motion arm joint. If the `goal` is out of these boundaries, "OUT OF BOUNDS" is printed on the serial monitor, and the function exits.

```cpp
if(goal > config.maxDegree || goal < config.minDegree){
  Serial.println("OUT OF BOUNDS");
  return;
}
```

Once the move is verified to be possible, the function prints out the name of the servo and its target angle.

```cpp
Serial.print("Moving Motor: ");
Serial.print(config.name);
Serial.print(" towards: ");
Serial.println(goal);
```

Next, we convert the `goal` angle into a pulse length that the servo understands. This is done through mapping the angle to its equivalent pulse length.

```cpp
goal = map(goal, config.minDegree, config.maxDegree, DFR_min, DFR_max);
```

Now the servo will actually move. We use the `setPWM()` method from the Adafruit library to execute the movement.

```cpp
pwm.setPWM(config.PWM_pin, 0, goal);
```

After the servo moves, the function waits for a short period and then checks the actual position reached, which is then printed out on the serial monitor.

```cpp
delay(1000);  // Wait for servo to reach position
Serial.println("Actual Angle: ");
Serial.println(getPos(config));
```

### Understanding the `getPos()` Function

The `getPos()` function helps us know the real-time position of the servo in degrees. 
This function reads the analog feedback from the servo's feedback pin and maps it to the corresponding angle. Sounds complicated? Let's break it down.

Here's the function:

```cpp
int getPos(const ServoConfig& config) {
  return abs(map(analogRead(config.Feedback_Pin), config.minFeedback, config.maxFeedback, config.minDegree, config.maxDegree));
}
```

#### The Role of `minFeedback` and `maxFeedback`

So, what's the deal with `minFeedback` and `maxFeedback`? These are the analog feedback values when the servo is at its minimum and maximum physical positions (angles). When you calibrate the servo, you store these values like this:

```cpp
config.minFeedback = analogRead(config.Feedback_Pin);  // At minimum angle
```
and
```cpp
config.maxFeedback = analogRead(config.Feedback_Pin);  // At maximum angle
```

These values help the `map()` function translate raw feedback into actual angles:

```cpp
map(analogRead(config.Feedback_Pin), config.minFeedback, config.maxFeedback, config.minDegree, config.maxDegree)
```

Imagine we're working with a servo for the base joint, the one that rotates the arm on the base, (`Base_conf`). After calibration, let's say the `minFeedback` value we got is `300`, and the `maxFeedback` value is `700`.

When the servo is at its minimum physical limit, the analog feedback reads `300`. When it's at its maximum, it reads `700`.

Now let's say you read a value of `500` from the feedback pin. Using `getPos()`, it would map this `500` like this:

```cpp
map(500, 300, 700, 0, 180)
```

It will tell you the servo is currently at `90 degrees`, right in the middle of its range from `0 to 180 degrees`.



