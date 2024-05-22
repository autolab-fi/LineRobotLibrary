#pragma once    
#include "Arduino.h"

#include <ESP32Encoder.h>
  
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
    ESP32Encoder encLeft;
    ESP32Encoder encRight;

public:
    
    //radius of wheels
    float radius_wheel;
    //distance between wheel and center
    float distance_between_wheel_and_center;
    //Coefficient for converting values from the encoder to an angle in degrees
    float encoder_degrees_optimal;
    // Coefficient for turning speed control
    float kp_rot;
    float ki_rot;
    float kd_rot;
    // Coefficient for robot motion straight. The coefficient is greater -> the lagging wheel turns more
    float k;
    lineRobot(int pinLeft1, int pinLeft2, int pinRight1, int pinRight2, float wheel_radius, float distance_between_wheels):encLeft(true, nullptr, nullptr),encRight(true, nullptr, nullptr){
	
        distance_between_wheel_and_center = distance_between_wheels/2;
        radius_wheel = radius_wheel;
        //setup left motor
        in1 = pinLeft1;
        in2 = pinLeft2;
        pinMode(in1,OUTPUT);
        pinMode(in2,OUTPUT);
        encLeft.attachHalfQuad(14, 27);
        //setup right motor
        in3 = pinRight1;
        in4 = pinRight2;
        pinMode(in3,OUTPUT);
        pinMode(in4,OUTPUT);
	    encRight.attachHalfQuad(18, 19);

        k=0.31;
        radius_wheel =wheel_radius/100;
        distance_between_wheel_and_center = distance_between_wheels/200;
        encoder_degrees_optimal = 2.96;
        ki_rot = 0.0002;
        kd_rot = 0.00001;
        kp_rot = 0.001;
        oldPositionLeft  = -999;
        oldPositionRight  = -999;
        leftPosition = 0;
        rightPosition = 0;
    };
    int computePID(float input, float setpoint, float kp, float kd, float ki, float dt, int minOut, int maxOut);
    void setRotateCoefficient_kp(float value);
    void setRotateCoefficient_ki(float value);
    void setRotateCoefficient_kd(float value);
    void setStraightMotionCoefficient(float value);
    void setEncoderDegrees(float value);
    void startMotorForwardLeft(int sp);
    void startMotorBackwardLeft(int sp);
    void startMotorForwardRight(int sp);
    void startMotorBackwardRight(int sp);
    void stopMotorLeft();
    void stopMotorRight();
    void stop();
    int changeDegrees(int ang);
    long moveMotors(int dir, int sp);
    void moveForwardSpeedDistance(int sp, float  dist);
    void moveBackwardSpeedDistance(int sp, float  dist);
    void moveForwardDistance(float  dist);
    void moveBackwardDistance(float  dist);
    void moveForwardSeconds(int seconds);
    void moveBackwardSeconds(int seconds);
    long getPositionLeftEncoder();
    long getPositionRightEncoder();
    long getPositionLeftEncoderDegrees();
    long getPositionRightEncoderDegrees();
    void turnLeftAngle(int ang);
    void turnRightAngle(int ang);
    void resetLeftEncoder();
    void resetRightEncoder();
    void resetLeftEncoderValue(int value);
    void resetRightEncoderValue(int value);
    void resetEncoders();
    void turnLeft();
    void turnRight();
    void rotate();
};

extern lineRobot robot;