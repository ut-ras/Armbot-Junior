# Armbot-Junior 

This repository contains all the necessary documentation and code base for the Armbot-Junior Demobots project.

## Introduction

Armbot-Junior is a miniature 6DOF (Degrees of Freedom) robotic arm project. It is primarily designed to serve as a test bed for the main Armbot project and as an educational tool. This project uses DFRobot's Servos with an internal analog feedback potentiometer, enabling the user to get real-time position data of the robot arm.

## Software Guide

Download the Arduino IDE either standalone or as a VSCode extension (or use platformIO if you're crazy). Clone the repository and open the project in the IDE. Be sure to install the Adafruit PWM, (`Adafruit_PWMServoDriver.h`), library from the Arduino Library Manager or elsewhere

1. [`ArmbotJr_DFR_servos_PWM_Shield.ino`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_servos_PWM_Shield.ino): The current working file. It's similar to `ArmbotJr_DFR_servos` but is designed for use with an Adafruit 16-Channel 12-bit PWM/Servo Driver Shield or a similar I2C device.

2. [`ArmbotJr_DFR_Functions.h`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_Functions.h): Header file, includes the needed libraries, contains the `ServoConfig` struct, the `Pose` struct, global variables and function prototypes for the `ArmbotJr_DFR_servos_PWM_Shield.ino` program.

3. [`ArmbotJr_DFR_Functions.cpp`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_Functions.cpp): Source file, contains the function  definitions for the `ArmbotJr_DFR_servos_PWM_Shield.ino` program.


This Arduino Sketch was originally in a single file, but was split into a "main" `.ino` file, a header file, and a source file to make it easier to read and understand.



## [`ArmbotJr_DFR_servos_PWM_Shield.ino`](ArmbotJr_DFR_servos_PWM_Shield/ArmbotJr_DFR_servos_PWM_Shield.ino) Explained

The `ArmbotJr_DFR_servos_PWM_Shield` program works as follows:




A struct named `ServoConfig` is defined to hold the configuration details of each servo such as name, PWM pin number, feedback pin number, a minimum and maximum degree value, and a minimum and maximum feedback value (for use in calibration). A constructor is also defined which accepts these parameters to create an object of the struct. 

###### (*A constructor is a special kind of function that gets called automatically when an object of the struct is created. Its main purpose is to assign initial values to the data members of the new object. It helps ensure that objects are valid as soon as they're created and simplifies code since initialization details are kept within the struct. Honestly this shouldnt really be done with a struct, I just kind of suck using Classes properly, add that to the TODO list lol*) 

### Global Variables and Constants 
- `const int mode`: A constant that determines the operating mode of the program. A value of `0` runs custom test code, while a value of `1` enables a "pseudo-multitasking" mode for running a sequence of poses. 
- `ServoConfig Base`: A `ServoConfig` object that holds the configuration details of the base servo.
- `ServoConfig J1`: A `ServoConfig` object that holds the configuration details of the J1 servo.
- `ServoConfig J2`: A `ServoConfig` object that holds the configuration details of the J2 servo.
- `ServoConfig J3`: A `ServoConfig` object that holds the configuration details of the J3 servo.
- `ServoConfig J4`: A `ServoConfig` object that holds the configuration details of the J4 servo.
- `ServoConfig allJoints[]`: An array of `ServoConfig` objects, each representing a joint in the robotic arm. 
- `const int numJoints`: Specifies the total number of joints, which is 5 in this case. 
- `Pose poseSequence[]`: An array of `Pose` objects, defining a sequence of poses to be executed. 
- `const int numPoses`: The total number of poses in `poseSequence`.
### `setup()` Function

The `setup` function performs the following tasks:
1. Initializes serial communication with a baud rate of 9600.
2. Starts the Adafruit PWM servo driver and sets its frequency to 60Hz. 
3. Locks all the motors in their current positions using `moveTo`. 
4. Calibrates all servos using the `calibrate` function.
### `loop()` Function

The `loop` function is divided into two primary sections, controlled by the `mode` variable:
#### Mode 1 (`mode == 1`) 
1. **Task 1 - Pose Execution** : 
- Uses the `millis` function to keep track of elapsed time. 
- Executes a pose from `poseSequence` every `intervalPose` milliseconds.
- Moves the servos to the positions specified in the next pose. 
- Cycles through the `poseSequence` array. 
2. **Task 2 - Print Joint Positions** : 
- Prints the current positions of all joints at intervals specified by `intervalPrint`.
#### Mode 0 (`mode == 0`)
- This is where you can add your own custom test code.
- The example code given moves joint J4 to -90 degrees and closes the claw, waits for 3 seconds, then moves J4 to 90 degrees and opens the claw, followed by another 3-second wait.
### Summary 
- **Mode 1** : Intended for "pseudo-multitasking" by executing a sequence of poses while continuously printing the joint positions. 
- **Mode 0** : Intended for running custom test code.

---
## Individual Structs and Functions Explained
### `ServoConfig` Struct

The `ServoConfig` struct serves as a configuration container for each servo in the robotic arm. It holds various parameters and states for a given servo: 
- `name`: A string to hold the name of the servo. 
- `PWM_Channel`: An integer representing the PWM pin number to which the servo is connected. 
- `Feedback_Pin`: An integer representing the pin number that receives feedback from the servo. 
- `minDegree` and `maxDegree`: Integers that specify the minimum and maximum limits of the servo in degrees. 
- `jointMinDegree` and `jointMaxDegree`: Integers that specify the minimum and maximum limits of the joint associated with the servo, in "joint space" (angles centered at zero, instead of 0 to 270 degrees, its -135 to +135 degrees, etc). 
- `minFeedback` and `maxFeedback`: Integers that specify the minimum and maximum feedback values that can be received from the servo. These get added in the `calibrate()` function.
- `defaultPos`: An integer that holds the default or "safe" position of the joint.

The struct also includes a constructor that initializes these fields.

At the top of the main code (.ino file), a `ServoConfig` object may be configured with:

```
ServoConfig J2("J2", 2, A2, 5, 270, -135, 135, 0);
```
Where `"J2"` is the joint's name, `2` is the PWM Channel, corresponding to where the servo cable is plugged into on the PWM Shield, `A2` is the analog input pin connected to servo's white feedback signal wire, `5, 270` are the minimum and maximum rotation/angle the J2 joint can physically move, and `-135, 135` are the minimum and maximum rotation/angle the J2 joint can move in "joint space" (centered at zero, instead of 0 to 270 degrees, its -135 to +135 degrees, etc). `0` is the default position of the joint (straight upwards) in this example.

You can configure the servo like this at the start of the code, or within the `setup()` function.

After configuring, you'll now be able to "pass in" `J2` to functions like `calibrate()`, `moveTo()`, and `getPos()`, etc.

---
### `Pose` Struct

The `Pose` struct is used for holding the states of all joints in a particular pose. It contains: 
- `jointStates`: An array of integers, where each index corresponds to a joint state in joint space. The last index (`jointStates[5]`) is used to indicate the state of the claw (open or closed).

The struct provides two constructors:
1. A default constructor that initializes all joint states to zero.
2. A parameterized constructor that allows setting of each joint state and the claw state (open or closed).

At the top of the main code (.ino file), or in the setup(), a `Pose` object may be configured with:

`Pose test_pose = Pose(0, -45, 90, -90, 90, open);
` 

Where `0, -45, 90, -90, 90` are the joint states in joint space, and `closed` is the claw state (open or closed).

later on, in the `loop()` function or elsewhere, you can then do:
```
  moveTo(Base, test_pose.jointStates[0]);
  moveTo(J1, test_pose.jointStates[1]);
  moveTo(J2, test_pose.jointStates[2]);
  moveTo(J3, test_pose.jointStates[3]);
  moveTo(J4, test_pose.jointStates[4]);
  BinaryClaw(test_pose.jointStates[5]);
```
This will move all the joints to the positions specified in the `test_pose` object.

###### (*This can def be made into it's own "moveToPose" function or something, but for now this works*)
---
### `calibrate(ServoConfig &config)`
#### Description

The `calibrate` function is responsible for calibrating a servo motor based on its configuration. Calibration involves finding the minimum and maximum feedback values and storing them in the `ServoConfig` object. It also moves the servo to its default position at the end of the process.
#### Parameters 
- `config`: A reference to a `ServoConfig` struct that contains the servo's configuration.
#### Implementation Details 
1. **Initialization** : It starts by printing the name of the servo being calibrated. 
2. **Pulse Length Calculation** : It calculates the pulse lengths (`pulseLen_min` and `pulseLen_max`) corresponding to the servo's minimum and maximum angles. These calculations use the global constants `DFR_min` and `DFR_max`. 
3. **Minimum Feedback Reading** : 
- The servo is first moved to its minimum angle using `pwm.setPWM()`. 
- After a delay, the minimum feedback value is read using `analogRead()` and stored in the `ServoConfig` object. 
4. **Maximum Feedback Reading** :
- The servo is then moved to its maximum angle.
- For joint "J1", it moves slowly to its max position for safety reasons. 
- Again, after a delay, the maximum feedback value is read and stored in the `ServoConfig` object. 
5. **Default Position** : Finally, the servo is moved to its default position using the `moveTo` function. If the movement fails, an error message is printed. 
6. **Return** : The function then returns, signaling the end of the calibration process.
#### Example Usage

Its best to do this in the `setup()` function:
```cpp
calibrate(Base);
calibrate(J1);
calibrate(J2);
//etc...
```
---
### `servoToJoint(int servo_angle, const ServoConfig &config)`
#### Description

This function translates a given servo angle to its corresponding angle in joint space.
#### Parameters 
- `servo_angle`: The angle of the servo in servo space. 
- `config`: A constant reference to a `ServoConfig` struct containing the servo's configuration. Since each joint has its own limits, this parameter is used to determine the joint space limits of the servo.
#### Implementation Details

The function uses the Arduino `map` function to translate the servo angle to a joint space angle based on the minimum and maximum degree limits defined in `ServoConfig`.
#### Example Usage

```cpp
int joint_angle = servoToJoint(90, J1);
```

---
### `jointToServo(int joint_angle, const ServoConfig &config)`
#### Description

This function performs the inverse operation of `servoToJoint`; it translates a given joint angle to its corresponding angle in servo space.
#### Parameters 
- `joint_angle`: The angle of the joint in joint space. 
- `config`: A constant reference to a `ServoConfig` struct containing the servo's configuration.
#### Implementation Details

Similar to `servoToJoint`, it also uses the `map` function but reverses the mapping to translate from joint space to servo space.
#### Example Usage

```cpp
int servo_angle = jointToServo(-45, J1);
```

---
### `getJointPos(const ServoConfig &config)`
#### Description

This function retrieves the current position of a servo in terms of joint space.
#### Parameters 
- `config`: A constant reference to a `ServoConfig` struct containing the servo's configuration.
#### Implementation Details 
- It first reads the current feedback from the servo using `analogRead`. 
- Then it maps this feedback to a servo angle based on the minimum and maximum feedback values defined in `ServoConfig`. 
- Finally, it uses `servoToJoint` to convert this servo angle to joint space.
#### Example Usage

```cpp
int current_joint_pos = getJointPos(J1);
```
---
### `moveTo(ServoConfig &config, int joint_goal)`
#### Description

The `moveTo` function is designed to move a servo to a specific position in joint space. It checks for boundary conditions, calculates the required pulse length, and attempts to move the servo.
#### Parameters 
- `config`: A reference to a `ServoConfig` struct containing the servo's configuration. 
- `joint_goal`: The desired joint position in joint space.
#### Implementation Details 
1. **Servo Goal Calculation** : It first translates the joint goal to servo space using `jointToServo`. 
2. **Boundary Check** : Checks if the calculated servo goal is within the min and max limits defined in `ServoConfig`. If not, it prints an "Out of bounds!!!" error and returns `false`. 
3. **Pulse Length Calculation** : It calculates the pulse length (`pulseLen`) required to move the servo to the desired position. 
4. **Servo Movement** : Uses `pwm.setPWM()` to move the servo to the desired position. 
5. **Position Verification (Commented Out)** : The function contains commented-out code for verifying if the servo has reached the desired position. This part is currently not implemented, which is mentioned in the comment. 
6. **Return** : Finally, the function returns `true`, indicating that the command to move the servo was issued.
#### Example Usage

```cpp
// Using the returned boolean value to check for success
bool success = moveTo(myServoConfig, -45);
if (!success) {
  Serial.println("Failed to move servo.");
}

// Without using the returned boolean value, simply issuing the command
moveTo(myServoConfig, -45);
```

---
### `BinaryClaw(int desired_claw_state)`
#### Description

The `BinaryClaw` function controls the state of the claw, setting it to either open or closed based on the `desired_claw_state` parameter. This function assumes that the claw's servo is connected to channel 5.
#### Parameters 
- `desired_claw_state`: An integer that indicates the desired state of the claw. The constant `closed` represents a closed state, while `open` represents an open state.
#### Implementation Details 
1. **Pulse Lengths** : The function has hardcoded pulse lengths (`close_PL` and `open_PL`), which were derived from testing, to open and close the claw. 
2. **State Check** : Although commented out, there's a provision to check whether the claw is already in the desired state. 
3. **Claw Movement** : 
- If `desired_claw_state` is `closed`, the function uses `pwm.setPWM()` to move the claw to the closed position. 
- If `desired_claw_state` is `open`, it moves the claw to the open position. 
4. **Logging** : The function prints the action it is taking ("Moving Claw to Closed/Open"). 
5. **Return** : Finally, the function returns `true` if it successfully issues the command to move the claw. Otherwise, it returns `false`.
#### Example Usage

```cpp
// Using the returned boolean value to check for success
bool success = BinaryClaw(closed);
if (!success) {
  Serial.println("Failed to move claw.");
}

// Simply issuing the command without checking for success
BinaryClaw(open);
```
---
### `printPos(const ServoConfig &config)`
#### Description

The `printPos` function prints the current position of a specified servo in joint space.
#### Parameters 
- `config`: A constant reference to a `ServoConfig` struct containing the servo's configuration.
#### Implementation Details 
- It prints the name of the servo followed by its current position in degrees, fetched using the `getJointPos` function.
#### Example Usage

```cpp
printPos(J1);  // Outputs something like "J1: -45 deg"
```

---
### `printAllJointPos(ServoConfig configs[], int numJoints)`
#### Description

The `printAllJointPos` function prints the current positions of all servos in an array in joint space.
#### Parameters 
- `configs`: An array of `ServoConfig` structs, each containing the configuration for one servo. 
- `numJoints`: The number of servos (or joints) in the array.
#### Implementation Details 
- It iterates through the `configs` array and prints the name of each servo along with its current position in joint space.
- The function also includes formatting to align the output neatly.
#### Example Usage

```cpp
ServoConfig allJoints[] = {Base, J1, J2, J3, J4};
printAllJointPos(allJoints, 5);  
// Outputs the current positions of all servos in the allJoints[] array
```
