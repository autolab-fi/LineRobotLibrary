#pragma once    
#include "Arduino.h"

#include <Encoder.h>
  
class lineRobot
{

private:
    int in1;
    int in2;
    int in3;
    int in4;
    int encoderLeftPin1;
    int encoderLeftPin2;
    int encoderRightPin1;
    int encoderRightPin2;
    long oldPositionLeft;
    long oldPositionRight;
    long leftPosition;
    long rightPosition;
    Encoder encLeft;
    Encoder encRight;
    

public:
    //radius of wheels
    float radius_wheel;
    //distance between wheel and center
    float distance_between_wheel_and_center;
    //Coefficient for converting values from the encoder to an angle in degrees
    float encoder_degrees_optimal;
    // Coefficient for turning speed control
    float k_rot;
    // Coefficient for robot motion straight. The coefficient is greater -> the lagging wheel turns more
    float k;
    lineRobot(int pinLeft1, int pinLeft2, int pinRight1, int pinRight2, int encoderLeftPin1, int encoderLeftPin2, int encoderRightPin1, int encoderRightPin2, float _radius_wheel, float distance_between_wheels):encLeft(encoderLeftPin1, encoderLeftPin2),encRight(encoderRightPin1, encoderRightPin2){
        distance_between_wheel_and_center = distance_between_wheels/2;
        radius_wheel = radius_wheel;
        in1 = pinLeft1;
        in2 = pinLeft2;
        in3 = pinRight1;
        in4 = pinRight2;
        //Setup left motor
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);

        //Setup right motor
        pinMode(in3, OUTPUT); 
        pinMode(in4, OUTPUT); 
        k=0.1;
        radius_wheel =_radius_wheel;
        distance_between_wheel_and_center = distance_between_wheels/2;
        encoder_degrees_optimal = 6.645;
        k_rot = 0.03;
        oldPositionLeft  = -999;
        oldPositionRight  = -999;
        leftPosition = 0;
        rightPosition = 0;
    };
    void set_rotate_coefficient(float value);
    void set_straight_motion_coefficient(float value);
    void set_encoder_degrees(float value);
    void startMotorForwardLeft(int sp);
    void startMotorBackwardLeft(int sp);
    void startMotorForwardRight(int sp);
    void startMotorBackwardRight(int sp);
    void stopMotorLeft();
    void stopMotorRight();
    long moveMotors(int dir, int sp);
    bool moveForwardSpeedDistance(int sp, float  dist);
    bool moveBackwardSpeedDistance(int sp, float  dist);
    bool moveForwardDistance(float  dist);
    bool moveBackwardDistance(float  dist);
    bool moveForwardSeconds(int seconds);
    bool moveBackwardSeconds(int seconds);
    long getPositionLeftEncoder();
    long getPositionRightEncoder();
    bool turnLeftAngle(int ang);
    bool turnRightAngle(int ang);
    void resetLeftEncoder();
    void resetRightEncoder();
    bool turnLeft();
    bool turnRight();
    bool rotate();
};

extern lineRobot robot;