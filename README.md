# Armbot-Junior

This repo contains all the necessary documentation and code base of the Armbot-Junior Demobots project. 

## Introduction

Armbot-Junior is a miniature 6DOF (Degrees of Freedom) robotic arm project which is primarily designed to serve as a test bed for the main Armbot project and as an educational tool. This project uses DFRobot's Servos with an analog feedback mechanism enabling the user to get real-time position data of the robot arm.

## Software Guide
There are three project folders at the moment, ```ArmbotJr_DFR_single_servo```, which implements a basic feedback calibration system and move-to-target command for a single DFRobot servo ```ArmbotJr_DFR_servos``` which adds a custom servo config struct that allows for easy configuration of individual parameters for each arm joint, and ```ArmbotJr_DFR_servos_PWM_Sheild```, the current working file. Similar to ```ArmbotJr_DFR_servos```, ```ArmbotJr_DFR_servos_PWM_Sheild``` implements the same/similar functionality but for use with an Adafruit 16-Channel 12-bit PWM/Servo Driver Shield (or similar I2C device). 

