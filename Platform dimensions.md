# Sverwe platform 

## Servo IDs

Two types of servos:
* in-wheel servo- responsible for rotating wheel (propelling robot)
* steering servo- controls yaw of each wheel independently allowing for independent wheel steering

In-wheel servo IDs:
* front left: 32
* front right: 35
* back left: 39
* back right: 36

Steering servos IDs:
* front left: 33
* front right: 34
* back left: 38
* back right: 37

## Servo limits:

Each of the in-wheel servos shouldn't have any explicit angular limit.
Each of the steering servos can rotate from -90deg to +90deg. Their center setting 0deg should reflect driving directly forward

## Platform dimensions

Dimensions can be accounted for 2 usecases:
* steering dimensions- then wheel YAW to wheel YAW axises are provided
* outer dimensions- for path planning and obstacle avoidance- teel max outer dimensions of the robot

Steering dimensions:
* length: 305mm
* width: 266.6mm

Outer dimensions:
* length: 470mm 
* width: 386mm

## Wheel radius

Each wheel has the same radius- 60mm.

## Position of the rplidar a1

Position of the RPlidar A1 scanner is given from central point between wheels projected to the ground below the robot:
* forward 150mm
* above: 200mm
* to the left: 40mm