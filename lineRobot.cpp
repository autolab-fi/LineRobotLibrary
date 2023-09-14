#include "lineRobot.h"


//lineRobot::lineRobot(int pinLeft1, int pinLeft2, int pinRight1, int pinRight2, int encoderLeftPin1, int encoderLeftPin2, int encoderRightPin1, int encoderRightPin2, float radius_wheel, float distance_between_wheels){
//    
//}
 void lineRobot::set_straight_motion_coefficient(float value){
    k = value;
  }
  void lineRobot::set_rotate_coefficient(float value){
    k_rot = value;
  }
  void lineRobot::set_encoder_degrees(float value){
    encoder_degrees_optimal = value;
  }
  void lineRobot::startMotorForwardLeft(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    analogWrite(in1, sp);
    digitalWrite(in2, LOW);
  }
  void lineRobot::startMotorBackwardLeft(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    digitalWrite(in1, LOW);
    analogWrite(in2, sp);
  }
  void lineRobot::startMotorForwardRight(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    analogWrite(in3, sp);
    digitalWrite(in4, LOW);
  }
  void lineRobot::startMotorBackwardRight(int sp){
    if (sp!=0)
        sp = map(sp, 0, 100, 43, 255);
    digitalWrite(in3, LOW);
    analogWrite(in4, sp);
  }
  void lineRobot::stopMotorLeft(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  void lineRobot::stopMotorRight(){
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  long lineRobot::moveMotors(int dir, int sp){
    if (sp>90)
        sp=90;
    leftPosition = abs(encLeft.read());
        
    rightPosition = abs(encRight.read());
    
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
  bool lineRobot::moveForwardSpeedDistance(int sp, float  dist){
      long res = 0;
      long t = millis();
      dist = dist*encoder_degrees_optimal*180/3.14/radius_wheel/100;
      while (res < dist){ 
       if (t<millis()){
          res = moveMotors(0, sp);
          t = millis()+3;
        }
      }
      stopMotorLeft();
      stopMotorRight();
      resetLeftEncoder();
      resetRightEncoder();
      return true;
  }
  
//Move backward for dist in cm 
bool lineRobot::moveBackwardSpeedDistance(int sp, float  dist){
      long res = 0; 
      long t = millis();
      dist = dist*encoder_degrees_optimal*180/3.14/radius_wheel/100;
      while (res < dist){ 
        if (t<millis()){
          res = moveMotors(1, sp);
          t = millis()+3;
        }
      }
      stopMotorLeft();
      stopMotorRight();
      resetLeftEncoder();
      resetRightEncoder();
      return true;
  }
  
  bool lineRobot::moveForwardDistance(float dist){
    return moveForwardSpeedDistance(1, dist);
  }
  
  bool lineRobot::moveBackwardDistance(float dist){
    return moveBackwardSpeedDistance(1, dist);
  }
  
//Move forward for seconds
  bool lineRobot::moveForwardSeconds(int seconds){
      long res = 0;
      long t = millis();
      long end_time = millis()+seconds*1000;
      while (millis() < end_time){ 
       if (t<millis()){
          res = moveMotors(0, 1);
          t = millis()+3;
        }
      }
      stopMotorLeft();
      stopMotorRight();
      resetLeftEncoder();
      resetRightEncoder();
      return true;
  }
  //Move forward for seconds
  bool lineRobot::moveBackwardSeconds(int seconds){
      long res = 0;
      long t = millis();
      long end_time = millis()+seconds*1000;
      while (millis() < end_time){ 
       if (t<millis()){
          res = moveMotors(1, 1);
          t = millis()+3;
        }
      }
      stopMotorLeft();
      stopMotorRight();
      resetLeftEncoder();
      resetRightEncoder();
      return true;
  }
  //retrun
  long lineRobot::getPositionLeftEncoder(){
    return encLeft.read();
  }
  long lineRobot::getPositionRightEncoder(){
    return encRight.read();
  }
  bool lineRobot::turnLeftAngle(int ang){
    long res = 0;
    long t = millis();
    long ang_goal = ang*distance_between_wheel_and_center/radius_wheel*encoder_degrees_optimal;
    //long ang_goal = get_angle_for_rotate(ang); 
    while ((leftPosition>ang_goal+10 or leftPosition<ang_goal-10) and (rightPosition<ang_goal-10 or rightPosition>ang_goal+10)){
      if (t<millis()){
          leftPosition = abs(encLeft.read());
          rightPosition = abs(encRight.read());
          if (oldPositionLeft != leftPosition or oldPositionRight != rightPosition){
          oldPositionLeft=leftPosition;
          oldPositionRight = rightPosition;
             int spr = abs(ang_goal-rightPosition)*k_rot;
             if (spr<1){
              spr=1;
             }
             if (spr>70){
              spr=70;
             }
             int spl = abs(ang_goal-leftPosition)*k_rot;
             if (spl<1){
              spl=1;
             } 
             if (spl>70){
              spl=70;
             }

             if (ang_goal>leftPosition){
              startMotorBackwardLeft(spl);
             } else{
              startMotorForwardLeft(spl);
             }
             if (ang_goal>rightPosition){
                startMotorForwardRight(spr);
             } else{
               startMotorBackwardRight(spr);
             }
          }
          t = millis()+2;
        }
    }
    resetLeftEncoder();
    resetRightEncoder();
    stopMotorLeft();
    stopMotorRight();
    delay(200);
    return true;
  }
    bool lineRobot::turnRightAngle(int ang){

    long res = 0;
    long t = millis();
    long ang_goal = ang*distance_between_wheel_and_center/radius_wheel*encoder_degrees_optimal;
    //long ang_goal = get_angle_for_rotate(ang); 
    while ((leftPosition>ang_goal+10 or leftPosition<ang_goal-10) and (rightPosition<ang_goal-10 or rightPosition>ang_goal+10)){
      if (t<millis()){
          leftPosition = abs(encLeft.read());
          rightPosition = abs(encRight.read());
          if (oldPositionLeft != leftPosition or oldPositionRight != rightPosition){
          oldPositionLeft=leftPosition;
          oldPositionRight = rightPosition;
             int spr = abs(ang_goal-rightPosition)*k_rot;
             if (spr<1){
              spr=1;
             }
             if (spr>70){
              spr=70;
             }
             int spl = abs(ang_goal-leftPosition)*k_rot;
             if (spl<1){
              spl=1;
             } 
             if (spl>70){
              spl=70;
             }
             if (ang_goal>leftPosition){
              startMotorForwardLeft(spl);
             } else{
              startMotorBackwardLeft(spl);
             }
             if (ang_goal>rightPosition){
                startMotorBackwardRight(spr);
             } else{
               startMotorForwardRight(spr);
             }
          }
          t = millis()+2;
        }
    }
    stopMotorLeft();
    stopMotorRight();
    resetLeftEncoder();
    resetRightEncoder();
    delay(200);
    return true;
  }
  void lineRobot::resetLeftEncoder(){
    leftPosition = 0;
    encLeft.write(0);
  }
  void lineRobot::resetRightEncoder(){
    rightPosition = 0;
    encRight.write(0);
  }
    bool lineRobot::turnLeft(){
    return turnLeftAngle(90);
  }
  bool lineRobot::turnRight(){
    return turnRightAngle(90);
  }
  bool lineRobot::rotate(){
    return turnRightAngle(180);
  }


lineRobot robot = lineRobot(4, 5, 6, 7, 18, 19, 20, 21, 0.0325, 0.194);