#pragma once    
#include "Arduino.h"

// #include <ESP32Encoder.h>
  
class lineRobot
{
public:
     uint8_t in1;
    uint8_t in2;
    uint8_t in3;
    uint8_t in4;
    //radius of wheels
    float RADIUS_WHEEL;
    float targetAngle = 0;
    //distance between wheel and center
    float distance_between_wheel_and_center;
    float kpAng;
    float kiAng;
    float kdAng;

    float kpSpeedLeft;
    float kpSpeedRight;
    float kiSpeed;
    float kdSpeedLeft;
    float kdSpeedRight;

    int leftMotorSignal;
    int rightMotorSignal;
    uint8_t STANDARD_SPEED_PERCENTAGE;
    uint8_t STANDARD_SPEED_PERCENTAGE_SLOW;
    long lastTimeLeft=0;
    long lastTimeRight=0;

    long lastTimeLeftSpeed=0;
    long lastTimeRightSpeed=0;

    float kStraight;

    float integralSpeedLeft=0;
    float previousErrSpeedLeft=0;
    float integralSpeedRight=0;
    float previousErrSpeedRight=0;

    float integralAngLeft=0;
    float previousErrAngLeft=0;
    float integralAngRight=0;
    float previousErrAngRight=0;

    int computePidSpeedMotor(float err, float kp, float kd, float ki, float& integral, float& previousErr, long& lastTime);
    int computePidAngleMotor(float err, float kp, float kd, float ki, float& integral, float& previousErr, long& lastTime);
    lineRobot();
    lineRobot(uint8_t leftMotorPin1, uint8_t leftMotorPin2, uint8_t rightMotorPin1, uint8_t rightMotorPin2, uint8_t encoderPinALeft,uint8_t encoderPinBLeft, uint8_t encoderPinARight, uint8_t encoderPinBRight, float wheel_radius, float distance_between_wheels, int encoderResolution);
    void resetRegulators();
    int encoderDegreesRight();
    int encoderDegreesLeft();
    float encoderRadianRight();
    float encoderRadianLeft();
    void getSpeedMotors(uint8_t interval, float& speedR,float& speedL);
    void runMotorsSpeed(int speedLeft, int speedRight);

    void runMotorSpeedLeft(int sp);
    void runMotorSpeedRight(int sp);
    void runMotorSpeedRight(int speed, float curSpeed);
    void runMotorSpeedLeft(int speed, float curSpeed);
    void runMotorRight(int u);
    void runMotorLeft(int u);
    float get_speed_L(uint8_t interval);
    float get_speed_R(uint8_t interval);

    void setBlockTrue();

    void stopMotorLeft();
    void stopMotorRight();
    void stop();
    void moveForwardSpeedDistance(int sp, float  dist);
    void moveBackwardSpeedDistance(int sp, float  dist);
    void moveForwardDistance(float  dist);
    void moveBackwardDistance(float  dist);
    void moveForwardSeconds(int seconds);
    void moveBackwardSeconds(int seconds);
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
    
    static void updateEncoderRight();
    void begin();

    static void updateEncoderLeft();
    static uint8_t encoderPinALeft;
    static uint8_t encoderPinBLeft;
    static uint8_t encoderPinARight;
    static uint8_t encoderPinBRight;
    static volatile long encoderPositionRight;
    static volatile int lastEncoded_R;
    static volatile long encoderPositionLeft;
    static volatile int lastEncoded_L;

    private:
        bool block=false;
        int pulsesPerRevolution;
        float k_speed_radians;
        float max_speed_radians;
    


};

extern lineRobot robot;
