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

  int lineRobot::changeDegrees(int angle){
      /*if (angle>270)
      return angle+16;
      if (angle>170)
      return angle+10;
      if (angle>160)
      return angle+9;
      if (angle>120)
      return angle+8;
      if (angle>110)
      return angle+6;
      if (angle>100)
      return angle+5;
      if (angle>90)
      return angle+3;
      if (angle>75)
      return angle+2;
      if (angle>60)
      return angle+1;
      if (angle>45)
     return angle;*/
     return angle;
  }

  void lineRobot::startMotorForwardLeft(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 65, 255);
    analogWrite(in1, sp);
    analogWrite(in2, 0);
  }
  void lineRobot::startMotorBackwardLeft(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 65, 255);
    analogWrite(in1, 0);
    analogWrite(in2, sp);
  }
  void lineRobot::startMotorForwardRight(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 65, 255);
    analogWrite(in4, 0);
    analogWrite(in3, sp);
  }
  void lineRobot::startMotorBackwardRight(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 65, 255);
    analogWrite(in4, sp);
    analogWrite(in3, 0);
  }
  void lineRobot::stopMotorLeft(){
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
  void lineRobot::stopMotorRight(){
    analogWrite(in3, 0);
    analogWrite(in4, 0);
  }
  void lineRobot::stop(){
    if (abs(encLeft.getCount())>abs(encRight.getCount())){
      stopMotorLeft();
      stopMotorRight();
    }
    else{
      stopMotorRight();
      stopMotorLeft();
    }
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
                if (leftPosition<rightPosition){
                startMotorForwardLeft(spl);
                startMotorForwardRight(spr);}
                else{
                  startMotorForwardRight(spr);
                  startMotorForwardLeft(spl);
                }
                break;
            case 1: 
                if (leftPosition<rightPosition){
                startMotorBackwardLeft(spl);
                startMotorBackwardRight(spr); }
                else{
                  startMotorBackwardRight(spr);
                  startMotorBackwardLeft(spl);
                }
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
  resetEncoders();
      long res = 0;
      long t = millis();
      dist = dist*encoder_degrees_optimal*180/3.14/radius_wheel/100;
      while (res < dist){ 
       if (t<millis()){
          res = moveMotors(0, sp);
          t = millis()+3;
        }
      }
      stop();
    resetEncoders();
  }
  
//Move backward for dist in cm 
void lineRobot::moveBackwardSpeedDistance(int sp, float  dist){
  resetEncoders();
      long res = 0; 
      long t = millis();
      dist = dist*encoder_degrees_optimal*180/3.14/radius_wheel/100;
      while (res < dist){ 
        if (t<millis()){
          res = moveMotors(1, sp);
          t = millis()+3;
        }
      }
      stop();
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
      resetEncoders();
      long res = 0;
      long t = millis();
      long end_time = millis()+seconds*1000;
      while (millis() < end_time){ 
       if (t<millis()){
          res = moveMotors(0, 1);
          t = millis()+3;
        }
      }
      stop();
    resetEncoders();
  }
  //Move forward for seconds
  void lineRobot::moveBackwardSeconds(int seconds){
    resetEncoders();
      long res = 0;
      long t = millis();
      long end_time = millis()+seconds*1000;
      while (millis() < end_time){ 
       if (t<millis()){
          res = moveMotors(1, 1);
          t = millis()+3;
        }
      }
      stop();
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
    resetEncoders();
    long res = 0;
    long t = millis();
    int error = 5;
    long ang_goal = changeDegrees(ang)*distance_between_wheel_and_center/radius_wheel*encoder_degrees_optimal;
    //long ang_goal = get_angle_for_rotate(ang); 
    while ((leftPosition>ang_goal+error or leftPosition<ang_goal-error) and (rightPosition<ang_goal-error or rightPosition>ang_goal+error)){
      if (t<millis()){
          leftPosition = abs(encLeft.getCount());
          rightPosition = abs(encRight.getCount());
          if (oldPositionLeft != leftPosition or oldPositionRight != rightPosition){
          oldPositionLeft = leftPosition;
          oldPositionRight = rightPosition;
             int spr = computePID(rightPosition, ang_goal, kp_rot, kd_rot, ki_rot, 0.002, 1, 70);
             int spl = computePID(leftPosition,  ang_goal, kp_rot, kd_rot, ki_rot, 0.002, 1, 70);
             if (rightPosition>leftPosition){
             if (ang_goal+error>leftPosition){
              startMotorBackwardLeft(spl);
             }else if (ang_goal-error<leftPosition){
              startMotorForwardLeft(spl);
             }
             if (ang_goal+error>rightPosition){
                startMotorForwardRight(spr);
             } else if (ang_goal-error<rightPosition){
               startMotorBackwardRight(spr);
             }
             }
             else {
              if (ang_goal+error>rightPosition){
                  startMotorForwardRight(spr);
              } else if (ang_goal-error<rightPosition){
                startMotorBackwardRight(spr);
              }
              if (ang_goal+error>leftPosition){
                startMotorBackwardLeft(spl);
              }else if (ang_goal-error<leftPosition){
                startMotorForwardLeft(spl);
              }
             }
          }
          t = millis()+2;
        }
    }
    stop();
    resetEncoders();
    delay(500);
  }
  void lineRobot::turnRightAngle(int ang){
    resetEncoders();
    long res = 0;
    long t = millis();
    long ang_goal = changeDegrees(ang)*distance_between_wheel_and_center/radius_wheel*encoder_degrees_optimal;
    int error = 5;
    //long ang_goal = get_angle_for_rotate(ang); 
    while ((leftPosition>ang_goal+error or leftPosition<ang_goal-error) and (rightPosition<ang_goal-error or rightPosition>ang_goal+error)){
      if (t<millis()){
          leftPosition = abs(encLeft.getCount());
          rightPosition = abs(encRight.getCount());
          if (oldPositionLeft != leftPosition or oldPositionRight != rightPosition){
          oldPositionLeft = leftPosition;
          oldPositionRight = rightPosition;
             int spr = computePID(rightPosition, ang_goal, kp_rot, kd_rot, ki_rot, 0.002, 1, 70);
             int spl = computePID(leftPosition,  ang_goal,  kp_rot, kd_rot, ki_rot, 0.002, 1, 70);
             if (rightPosition>leftPosition){
              if (ang_goal+error>leftPosition){
                startMotorForwardLeft(spl);
              } else if (ang_goal-error<leftPosition) {
                startMotorBackwardLeft(spl);
              }
              if (ang_goal+error>rightPosition){
                  startMotorBackwardRight(spr);
              } else if (ang_goal-error<rightPosition) {
                startMotorForwardRight(spr);
              }
             }
             else{
              if (ang_goal+error>rightPosition){
                startMotorBackwardRight(spr);
              } else if (ang_goal-error<rightPosition) {
                startMotorForwardRight(spr);
              }
              if (ang_goal+error>leftPosition){
                startMotorForwardLeft(spl);
              } else if (ang_goal-error<leftPosition) {
                startMotorBackwardLeft(spl);
              }
             }
          }
          t = millis()+2;
        }
    }
    stop();
    resetEncoders();
    delay(500);
  }
  void lineRobot::resetLeftEncoder(){
    leftPosition = 0;
    encLeft.clearCount();
  }
  void lineRobot::resetRightEncoder(){
    rightPosition = 0;
    encRight.clearCount();
  }
  void lineRobot::resetLeftEncoderValue(int value){
    leftPosition = value;
    encLeft.setCount(value);
    //encLeft.clearCount();
  }
  void lineRobot::resetRightEncoderValue(int value){
    rightPosition = value;
    encRight.setCount(value);
    //encRight.clearCount();
  }
  void lineRobot::resetEncoders(){
    /*long delta = abs(encLeft.getCount())-abs(encRight.getCount());
    if (delta>5){
      resetLeftEncoderValue(delta);
      resetRightEncoder();
      return;
    }
    if (delta<-5){
      resetRightEncoderValue(delta);
      resetLeftEncoder();
      return;
    }*/
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


lineRobot robot = lineRobot(2, 4, 12, 13, 3.25, 20.5);