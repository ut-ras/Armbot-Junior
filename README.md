# Armbot-Junior

This repository contains all the necessary documentation and code base for the Armbot-Junior Demobots project.

## Introduction

Armbot-Junior is a miniature 6DOF (Degrees of Freedom) robotic arm project. It is primarily designed to serve as a test bed for the main Armbot project and as an educational tool. This project uses DFRobot's Servos with an analog feedback mechanism, enabling the user to get real-time position data of the robot arm.

## Software Guide

Currently, there are three project folders:

1. `ArmbotJr_DFR_single_servo`: Implements a basic feedback calibration system and move-to-target command for a single DFRobot servo.
2. `ArmbotJr_DFR_servos`: Adds a custom servo config struct that allows for easy configuration of individual parameters for each arm joint.
3. `ArmbotJr_DFR_servos_PWM_Shield`: The current working file. It's similar to `ArmbotJr_DFR_servos` but is designed for use with an Adafruit 16-Channel 12-bit PWM/Servo Driver Shield or a similar I2C device.

## ArmbotJr_DFR_servos_PWM_Shield

The `ArmbotJr_DFR_servos_PWM_Shield` program works as follows:

A struct named `ServoConfig` is defined to hold the configuration details of each servo such as name, PWM pin number, feedback pin number, a minimum and maximum degree value, and a minimum and maximum feedback value (for use in calibration). A constructor is also defined which accepts these parameters to create an object of the struct. 

###### (*A constructor is a special kind of function that gets called automatically when an object of the struct is created. Its main purpose is to assign initial values to the data members of the new object. It helps ensure that objects are valid as soon as they're created and simplifies code since initialization details are kept within the struct.*)


```cpp
struct ServoConfig {
  char name[20];
  int PWM_pin;
  int Feedback_Pin;
  int minDegree;
  int maxDegree;
  int minFeedback;
  int maxFeedback;
};
```


As of writing, only two servos are initialized using the `ServoConfig struct` - "Base" and "J1".

***Please note, "JX" is just a placeholder for additional servos that are yet to be added.***

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


In the `setup()` function, the serial communication begins with a baud rate of 9600. The PWM servo driver is initialized and its frequency is set to 60 Hz. The two servos are calibrated by calling the `calibrate()` function.

### The `moveTo()` Function Explained

`moveTo(ServoConfig &config, int goal)` function is used to move a servo to a target position. It checks if the target position is within the servo's valid degree range. If yes, it maps the degree to a pulse length and moves the servo to the target position.

The function starts by ensuring the angle you want to move the servo to (`goal`) is within the allowable range. If the `goal` is out of these boundaries, "OUT OF BOUNDS" is printed on the serial monitor, and the function exits.

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

The `getPos()` function is a game-changer—it helps you know the real-time position of the servo in degrees. This function reads the analog feedback from the servo's feedback pin and maps it to the corresponding angle. Sounds complicated? Let's break it down.

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


### The `loop()` Function

Finally, the `loop()` function is where you put your `moveTo()` function calls. For example:

```cpp
void loop() {
  moveTo(Base, Base_conf, 0);
  moveTo(Base, Base_conf, 180);
}
```

In this example, the `Base` servo moves between 0 and 180 degrees.
