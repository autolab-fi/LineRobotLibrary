#include "lineRobot.h"


 void lineRobot::setStraightMotionCoefficient(float value){
    k = value;
  }
  void lineRobot::setRotateCoefficient_kp(float value){
    kp_rot = value;
  }
  void lineRobot::setRotateCoefficient_ki(float value){
    ki_rot = value;
  }
  void lineRobot::setRotateCoefficient_kd(float value){
    kd_rot = value;
  }
  void lineRobot::setEncoderDegrees(float value){
    encoder_degrees_optimal = value;
  }

  void lineRobot::startMotorForwardLeft(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    analogWrite(in1, 0);
    analogWrite(in2, sp);
  }
  void lineRobot::startMotorBackwardLeft(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    analogWrite(in1,sp);
    analogWrite(in2,0);
  }
  void lineRobot::startMotorForwardRight(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    analogWrite(in4, 0);
    analogWrite(in3, sp);
  }
  void lineRobot::startMotorBackwardRight(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    analogWrite(in4, sp);
    analogWrite(in3, 0);
  }
  void lineRobot::stopMotorLeft(){
    analogWrite(in1,0);
    analogWrite(in2,0);
  }
  void lineRobot::stopMotorRight(){
    analogWrite(in3,0);
    analogWrite(in4,0);
  }
  void lineRobot::stopRobot(){
    stopMotorLeft();
    stopMotorRight();
  }

  long lineRobot::moveMotors(int dir, int sp){
    if (sp>90)
        sp=90;
    leftPosition = abs(encLeft.getCount());
        
    rightPosition = abs(encRight.getCount());
    
    if (oldPositionLeft != leftPosition or oldPositionRight != rightPosition){
        oldPositionLeft=leftPosition;
        oldPositionRight = rightPosition;
        int delta = (rightPosition-leftPosition)*k;
        int spl = sp;
        int spr = sp;
        if (delta>0){
            spl+=abs(delta);
        }else{
            spr+=abs(delta);
        }
        switch (dir){
            case 0: 
                startMotorForwardLeft(spl);
                startMotorForwardRight(spr); 
                break;
            case 1: 
                startMotorBackwardLeft(spl);
                startMotorBackwardRight(spr); 
                break;
            default:
                break;
        }  
    }
    long result = (leftPosition+rightPosition)/2;
    return result;
  }
  //Move forward for dist in cm 
void lineRobot::moveForwardSpeedDistance(int sp, float  dist){
      long res = 0;
      long t = millis();
      dist = dist*encoder_degrees_optimal*180/3.14/radius_wheel/100;
      while (res < dist){ 
       if (t<millis()){
          res = moveMotors(0, sp);
          t = millis()+3;
        }
      }
      stopRobot();
    resetEncoders();
  }
  
//Move backward for dist in cm 
void lineRobot::moveBackwardSpeedDistance(int sp, float  dist){
      long res = 0; 
      long t = millis();
      dist = dist*encoder_degrees_optimal*180/3.14/radius_wheel/100;
      while (res < dist){ 
        if (t<millis()){
          res = moveMotors(1, sp);
          t = millis()+3;
        }
      }
      stopRobot();
    resetEncoders();
  }
  
  void lineRobot::moveForwardDistance(float dist){
    moveForwardSpeedDistance(1, dist);
  }
  
  void lineRobot::moveBackwardDistance(float dist){
    moveBackwardSpeedDistance(1, dist);
  }
  
//Move forward for seconds
  void lineRobot::moveForwardSeconds(int seconds){
      long res = 0;
      long t = millis();
      long end_time = millis()+seconds*1000;
      while (millis() < end_time){ 
       if (t<millis()){
          res = moveMotors(0, 1);
          t = millis()+3;
        }
      }
      stopRobot();
    resetEncoders();
  }
  //Move forward for seconds
  void lineRobot::moveBackwardSeconds(int seconds){
      long res = 0;
      long t = millis();
      long end_time = millis()+seconds*1000;
      while (millis() < end_time){ 
       if (t<millis()){
          res = moveMotors(1, 1);
          t = millis()+3;
        }
      }
      stopRobot();
    resetEncoders();
  }
  //retrun
  long lineRobot::getPositionLeftEncoder(){
    return encLeft.getCount();
  }
  long lineRobot::getPositionRightEncoder(){
    return encRight.getCount();
  }
  long lineRobot::getPositionLeftEncoderDegrees(){
    return long(encLeft.getCount()/encoder_degrees_optimal);
  }
  long lineRobot::getPositionRightEncoderDegrees(){
    return long(encRight.getCount()/encoder_degrees_optimal);
  }
  int lineRobot::computePID(float input, float setpoint, float kp, float kd, float ki, float dt, int minOut, int maxOut) {
    float err = setpoint - input;
    static float integral = 0, prevErr = 0;
    integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
    float D = (err - prevErr) / dt;
    prevErr = err;
    return constrain(err * kp + integral + D * kd, minOut, maxOut);
  }
  void lineRobot::turnLeftAngle(int ang){
    long res = 0;
    long t = millis();
    long ang_goal = ang*distance_between_wheel_and_center/radius_wheel*encoder_degrees_optimal;
    //long ang_goal = get_angle_for_rotate(ang); 
    while ((leftPosition>ang_goal+10 or leftPosition<ang_goal-10) and (rightPosition<ang_goal-10 or rightPosition>ang_goal+10)){
      if (t<millis()){
          leftPosition = abs(encLeft.getCount());
          rightPosition = abs(encRight.getCount());
          if (oldPositionLeft != leftPosition or oldPositionRight != rightPosition){
          oldPositionLeft = leftPosition;
          oldPositionRight = rightPosition;
             int spr = computePID(rightPosition, ang_goal, kp_rot, kd_rot, ki_rot, 0.002, 1, 70);
             int spl = computePID(leftPosition,  ang_goal, kp_rot, kd_rot, ki_rot, 0.002, 1, 70);

             if (ang_goal+8>leftPosition){
              startMotorBackwardLeft(spl);
             }else if (ang_goal-8<leftPosition){
              startMotorForwardLeft(spl);
             }
             if (ang_goal+8>rightPosition){
                startMotorForwardRight(spr);
             } else if (ang_goal-8<rightPosition){
               startMotorBackwardRight(spr);
             }
          }
          t = millis()+2;
        }
    }
    stopRobot();
    resetEncoders();
    delay(500);
  }
  void lineRobot::turnRightAngle(int ang){
    long res = 0;
    long t = millis();
    long ang_goal = ang*distance_between_wheel_and_center/radius_wheel*encoder_degrees_optimal;
    //long ang_goal = get_angle_for_rotate(ang); 
    while ((leftPosition>ang_goal+10 or leftPosition<ang_goal-10) and (rightPosition<ang_goal-10 or rightPosition>ang_goal+10)){
      if (t<millis()){
          leftPosition = abs(encLeft.getCount());
          rightPosition = abs(encRight.getCount());
          if (oldPositionLeft != leftPosition or oldPositionRight != rightPosition){
          oldPositionLeft = leftPosition;
          oldPositionRight = rightPosition;
             int spr = computePID(rightPosition, ang_goal, kp_rot, kd_rot, ki_rot, 0.002, 1, 70);
             int spl = computePID(leftPosition,  ang_goal,  kp_rot, kd_rot, ki_rot, 0.002, 1, 70);
             if (ang_goal+10>leftPosition){
              startMotorForwardLeft(spl);
             } else if (ang_goal-10<leftPosition) {
              startMotorBackwardLeft(spl);
             }
             if (ang_goal+10>rightPosition){
                startMotorBackwardRight(spr);
             } else if (ang_goal-10<rightPosition) {
               startMotorForwardRight(spr);
             }
          }
          t = millis()+2;
        }
    }
    stopRobot();
    resetEncoders();
    delay(500);
  }
  void lineRobot::resetLeftEncoder(){
    leftPosition = 0;
    encLeft.setCount (0);
  }
  void lineRobot::resetRightEncoder(){
    rightPosition = 0;
    encRight.setCount (0);
  }
  void lineRobot::resetEncoders(){
    resetRightEncoder();
    resetLeftEncoder();
  }
  void lineRobot::turnLeft(){
    return turnLeftAngle(90);
  }
  void lineRobot::turnRight(){
    return turnRightAngle(90);
  }
  void lineRobot::rotate(){
    return turnRightAngle(180);
  }


lineRobot robot = lineRobot(2, 4, 12, 13, 2.88, 22);