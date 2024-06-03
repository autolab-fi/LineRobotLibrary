#include "lineRobot.h"


// void lineRobot::setStraightMotionCoefficient(float value){
//   k = value;
// }
//lineRobot* lineRobot::instance = nullptr;

uint8_t lineRobot::encoderPinALeft=14;
uint8_t lineRobot::encoderPinBLeft=27;
uint8_t lineRobot::encoderPinARight=18;
uint8_t lineRobot::encoderPinBRight=19;
volatile long lineRobot::encoderPositionRight=0;
volatile int lineRobot::lastEncoded_R=0;
volatile long lineRobot::encoderPositionLeft=0;
volatile int lineRobot::lastEncoded_L=0;


lineRobot::lineRobot(uint8_t leftMotorPin1, uint8_t leftMotorPin2, uint8_t rightMotorPin1, 
  uint8_t rightMotorPin2, uint8_t encoderPinALeft_,uint8_t encoderPinBLeft_, 
  uint8_t encoderPinARight_, uint8_t encoderPinBRight_, float wheel_radius, float distance_between_wheels, int encoderResolution)
  //funcPointerLeft(&lineRobot::updateEncoderLeft),
  //funcPointerRight(&lineRobot::updateEncoderRight)
{
	
        distance_between_wheel_and_center = distance_between_wheels/2;
        RADIUS_WHEEL = wheel_radius;
        //setup left motor
        in1 = leftMotorPin1;
        in2 = leftMotorPin2;
        pinMode(in1,OUTPUT);
        pinMode(in2,OUTPUT);
        // encLeft.attachHalfQuad(14, 27);
        //setup right motor
        in3 = rightMotorPin1;
        in4 = rightMotorPin2;
        pinMode(in3,OUTPUT);
        pinMode(in4,OUTPUT);
	    // encRight.attachHalfQuad(18, 19);
        // encoderPositionRight = 0;
        // lastEncoded_R = 0;
        //encoderPositionLeft = 0;
        //lastEncoded_L = 0;
        rightMotorSignal = 0;
        leftMotorSignal = 0;
        pulsesPerRevolution = encoderResolution;
        max_speed_radians = 15;
        k_speed_radians = max_speed_radians/100.0;

        kpAng = 55.00;
        kiAng = 100.00;
        kdAng = 2.50;
        kpSpeedLeft = 55.00;
        kpSpeedRight = 35.00;
        kdSpeedLeft = 0.60;
        kdSpeedRight = 0.15;
        kiSpeed = 0.00;
        kStraight = 120.00;
            
        STANDARD_SPEED_PERCENTAGE = 60;

        encoderPinALeft = 14;
        encoderPinBLeft = 27;
        encoderPinARight = 18;
        encoderPinBRight = 19;

    }



void lineRobot::begin(){
  pinMode(encoderPinALeft, INPUT);
  pinMode(encoderPinBLeft, INPUT);
  pinMode(encoderPinARight, INPUT);
  pinMode(encoderPinBRight, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinALeft), lineRobot::updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinBLeft), lineRobot::updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinARight), lineRobot::updateEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinBRight), lineRobot::updateEncoderRight, CHANGE);
}    

void lineRobot::updateEncoderLeft() {
  // Чтение текущих значений пинов A и B
  //Serial.println("AAAAAAAAAAAAAA");
  int MSB_L = digitalRead(encoderPinALeft); // Most Significant Bit
  int LSB_L = digitalRead(encoderPinBLeft); // Least Significant Bit
  //Составление двухбитного числа из значений пинов A и B
  int encoded_L = (MSB_L << 1) | LSB_L;

  // Определение изменений состояния энкодера
  int sum_L = (lastEncoded_L << 2) | encoded_L;

  // Обновление позиции энкодера в зависимости от направления вращения

  if (sum_L == 0b1101 || sum_L == 0b0100 || sum_L == 0b0010 || sum_L == 0b1011) encoderPositionLeft++;
  if (sum_L == 0b1110 || sum_L == 0b0111 || sum_L == 0b0001 || sum_L == 0b1000) encoderPositionLeft--;

  // Сохранение текущего состояния для следующего прерывания
  lastEncoded_L = encoded_L;
}

void lineRobot::updateEncoderRight() {
  // Чтение текущих значений пинов A и B
  int MSB_R = digitalRead(encoderPinARight); // Most Significant Bit
  int LSB_R = digitalRead(encoderPinBRight); // Least Significant Bit
  // Составление двухбитного числа из значений пинов A и B
  int encoded_R = (MSB_R << 1) | LSB_R;

  // Определение изменений состояния энкодера
  int sum_R = (lastEncoded_R << 2) | encoded_R;


  // Обновление позиции энкодера в зависимости от направления вращения
  if (sum_R == 0b1101 || sum_R == 0b0100 || sum_R == 0b0010 || sum_R == 0b1011) encoderPositionRight--;
  if (sum_R == 0b1110 || sum_R == 0b0111 || sum_R == 0b0001 || sum_R == 0b1000) encoderPositionRight++;

  // Сохранение текущего состояния для следующего прерывания
  lastEncoded_R = encoded_R;
}


void lineRobot::moveLeftMotorSpeed(int speed){
  float cur_sp = get_speed_L(100);
  moveLeftMotorSpeed(speed,cur_sp);
}

void lineRobot::moveLeftMotorSpeed(int speed, float curSpeed){

  float target_speed = speed*k_speed_radians;
  float err = target_speed - curSpeed;
  //int u = computePidSpeedMotor(err, kp, kd, ki, integral_L, previousErr_L); 
  leftMotorSignal=computePidSpeedMotor(err, kpSpeedLeft, kdSpeedLeft, kiSpeed, integralSpeedLeft, previousErrSpeedLeft, lastTimeLeftSpeed);
  moveLeftMotor(leftMotorSignal);
}


void lineRobot::moveRightMotorSpeed(int speed){
  float cur_sp = get_speed_R(100);
  moveRightMotorSpeed(speed,cur_sp);
}

void lineRobot::moveRightMotorSpeed(int speed, float curSpeed){
  //float cur_sp = get_speed_R(100);
  float target_speed = speed*k_speed_radians;
  float err = target_speed - curSpeed;
 // int u = ; 
  rightMotorSignal=computePidSpeedMotor(err, kpSpeedRight, kdSpeedRight, kiSpeed, integralSpeedRight, previousErrSpeedRight, lastTimeRightSpeed);
  moveRightMotor(rightMotorSignal);
}



void lineRobot::moveLeftMotor(int u){
  if (u<0){
    analogWrite(in1,0);
    analogWrite(in2,-u);
  }
  else {
    analogWrite(in1,u);
    analogWrite(in2,0);
  }
}

void lineRobot::moveRightMotor(int u){
  if (u<0){
    analogWrite(in3,0);
    analogWrite(in4,-u);
  }
  else {
    analogWrite(in3,u);
    analogWrite(in4,0);
  }
}


float lineRobot::get_speed_L(uint8_t interval){
  float lastPos = encoderRadianLeft();
  delay(interval);
  float curPos = encoderRadianLeft();
  float delta_pos = (curPos - lastPos);
  float delta_time = (float(interval) / 1000.0);
  return delta_pos / delta_time;
}
float lineRobot::get_speed_R(uint8_t interval){
  float lastPos = encoderRadianRight();
  delay(interval);
  float curPos = encoderRadianRight();
  float delta_pos = (curPos - lastPos);
  float delta_time = (float(interval) / 1000.0);
  return delta_pos / delta_time;
}
 void lineRobot::getSpeedMotors(uint8_t interval, float& speedL,float& speedR){
  float lastPosR = encoderRadianRight();
  float lastPosL = encoderRadianLeft();
  delay(interval);
  float curPosR = encoderRadianRight();
  float curPosL = encoderRadianLeft();
  float delta_posR = (curPosR - lastPosR);
  float delta_posL = (curPosL - lastPosL);
  float delta_time = (float(interval) / 1000.0);
  speedR = delta_posR/delta_time;
  speedL = delta_posL/delta_time;
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
  stopMotorRight();
  stopMotorLeft();
}



int lineRobot::computePidSpeedMotor(float err, float kp, float kd, float ki, float& integral, float& previousErr, long& lastTime) {
    // Пропорциональная составляющая
    float P = kp * err;
    float dt =(millis()-lastTime)/1000.0;
    lastTime = millis();
    // Интегральная составляющая
    if (P<25){
      integral += err*dt;
    }
    float I = ki * integral;

    // Дифференциальная составляющая
    float D = kd * (err - previousErr)/dt;

    // Обновляем предыдущую ошибку
    previousErr = err;

    // Вычисляем итоговое значение ПИД-регулятора
    float output = P + I + D;

    // Установить минимальный выход, если ошибка больше abs(0.1)
    // if (fabs(err) > 0.1 && fabs(output) < 1) {
    //     output = (output > 0) ? 1 : -1;
    // }

    // Преобразуем выходное значение в диапазон от -255 до 255
    int motorSpeed = constrain((int)output, -255, 255);

    return motorSpeed;
}

int lineRobot::computePidAngleMotor(float err, float kp, float kd, float ki, float& integral, float& previousErr, long& lastTime) {
  float P = kp * err;
  float dt =(millis()-lastTime)/1000.0; 
  // Интегральная составляющая
  if (P<35){
    integral += err*dt;
  }
  float I = ki * integral;

  // Дифференциальная составляющая
  float D = kd * (err - previousErr)/(dt);
  lastTime = millis();
  // Обновляем предыдущую ошибку  
  previousErr = err;

  // Вычисляем итоговое значение ПИД-регулятора
  float output = P + I + D;

  // Преобразуем выходное значение в диапазон от -255 до 255
  int motorSpeed = constrain((int)output, -100, 100);

  return motorSpeed;
}



//Move forward for dist in cm 
void lineRobot::moveForwardSpeedDistance(int sp, float  dist){
  // dist in cm, RADIUS_WHEEL in cm
  targetAngle = dist/(RADIUS_WHEEL)*1.02;
  resetEncoders();
  resetRegulators();
  float angleLeft = 0.0;
  float angleRight = 0.0;
  float k_sp = abs(sp)/100.0;
  long startTime = millis();
  unsigned int period = constrain(120*dist/k_sp+3000,0,30000);
  float curSpeedL = 0;
  float curSpeedR = 0;
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  previousErrAngLeft = targetAngle;
  previousErrAngRight = targetAngle;
  while ((abs(targetAngle-angleLeft)>0.2 || abs(targetAngle-angleRight)>0.2) && period>millis()-startTime) { 
    angleLeft = encoderRadianLeft();
    angleRight = encoderRadianRight();
    leftMotorSpeed = computePidAngleMotor(targetAngle-angleLeft, kpAng, kdAng, kiAng,integralAngLeft,previousErrAngLeft, lastTimeLeft);
    rightMotorSpeed = computePidAngleMotor(targetAngle-angleRight, kpAng, kdAng, kiAng,integralAngRight,previousErrAngRight, lastTimeRight);
    leftMotorSpeed = constrain(leftMotorSpeed*k_sp, -75, 75);
    rightMotorSpeed = constrain(rightMotorSpeed*k_sp, -75, 75);
    if (millis()-startTime<1000){
      float power = (millis()-startTime)/1000.0;
      leftMotorSpeed = leftMotorSpeed*constrain(power, 0.4, 1.0);
      rightMotorSpeed = rightMotorSpeed*constrain(power, 0.4, 1.0);
    }
    if (abs(previousErrAngLeft)<abs(previousErrAngRight)) {
        rightMotorSpeed += (angleLeft - angleRight) * kStraight; // Увеличиваем скорость правого мотора
    } else if (abs(previousErrAngLeft)>abs(previousErrAngRight)) {
        leftMotorSpeed += (angleRight - angleLeft) * kStraight; // Увеличиваем скорость левого мотора
    }
    // Движение моторов с вычисленными скоростями
    getSpeedMotors(50,curSpeedL, curSpeedR);
    moveLeftMotorSpeed(leftMotorSpeed, curSpeedL);
    moveRightMotorSpeed(rightMotorSpeed, curSpeedR);
  }
  stop();
  //resetEncoders();
}


//Move backward for dist in cm 
void lineRobot::moveBackwardSpeedDistance(int sp, float  dist){
    
    // dist in cm, RADIUS_WHEEL in cm
    targetAngle = dist/(RADIUS_WHEEL)*1.02;
    resetEncoders();
    resetRegulators();
    float angleLeft = 0.0;
    float angleRight = 0.0;
    float k_sp = abs(sp)/100.0;
    long startTime = millis();
    unsigned int period = constrain(120*dist/k_sp+3000,0,30000);
    previousErrAngLeft = targetAngle;
    previousErrAngRight = targetAngle;
    float curSpeedL = 0;
    float curSpeedR = 0;
    while ((abs(angleLeft-targetAngle)>0.2 || abs(angleRight-targetAngle)>0.2) && period>millis()-startTime) { 
      angleLeft = abs(encoderRadianLeft());
      angleRight = abs(encoderRadianRight());
       int leftMotorSpeed = computePidAngleMotor(targetAngle-angleLeft, kpAng, kdAng, kiAng,integralAngLeft,previousErrAngLeft, lastTimeLeft);
      int rightMotorSpeed = computePidAngleMotor(targetAngle-angleRight, kpAng, kdAng, kiAng,integralAngRight,previousErrAngRight, lastTimeRight);
     /// Ограничение скорости в диапазоне от -100 до 100
      leftMotorSpeed = constrain(leftMotorSpeed, -75, 75);
      rightMotorSpeed = constrain(rightMotorSpeed, -75, 75);
      if (millis()-startTime<1000){
        float power = (millis()-startTime)/1000.0;
        leftMotorSpeed = leftMotorSpeed*constrain(power, 0.4, 1.0);
        rightMotorSpeed = rightMotorSpeed*constrain(power, 0.4, 1.0);
      }
      // moveLeftMotor прямолинейности
      if (abs(previousErrAngLeft)<abs(previousErrAngRight)) {
          rightMotorSpeed += (angleLeft - angleRight) * kStraight; // Увеличиваем скорость правого мотора
      } else if (abs(previousErrAngLeft)>abs(previousErrAngRight)) {
          leftMotorSpeed += (angleRight - angleLeft) * kStraight; // Увеличиваем скорость левого мотора
      }

      getSpeedMotors(50,curSpeedL, curSpeedR);
      moveLeftMotorSpeed(-leftMotorSpeed*k_sp, curSpeedL);
      moveRightMotorSpeed(-rightMotorSpeed*k_sp, curSpeedR);
    }
    stop();
    //resetEncoders();
}

void lineRobot::moveForwardDistance(float dist){
  moveForwardSpeedDistance(STANDARD_SPEED_PERCENTAGE, dist);
}

void lineRobot::moveBackwardDistance(float dist){
  moveBackwardSpeedDistance(STANDARD_SPEED_PERCENTAGE, dist);
}

//Move forward for seconds
void lineRobot::moveForwardSeconds(int seconds){
    resetEncoders();
    resetRegulators();
    long startTime = millis();
    unsigned int period = seconds*1000;
    float angleLeft = 0;
    float angleRight = 0;
    float curSpeedL = 0;
    float curSpeedR = 0;
    while (period>millis()-startTime){ 
      angleLeft = encoderRadianLeft();
      angleRight = encoderRadianRight();
      int leftMotorSpeed = STANDARD_SPEED_PERCENTAGE;
      int rightMotorSpeed = STANDARD_SPEED_PERCENTAGE;
      if (millis()-startTime<1000){
        float power = (millis()-startTime)/1000.0;
        leftMotorSpeed = leftMotorSpeed*constrain(power, 0.4, 1.0);
        rightMotorSpeed = rightMotorSpeed*constrain(power, 0.4, 1.0);
      }
      if (angleLeft>angleRight) {
          rightMotorSpeed += (angleLeft - angleRight) * kStraight; // Увеличиваем скорость правого мотора
      } else if (angleLeft<angleRight) {
          leftMotorSpeed += (angleRight - angleLeft) * kStraight; // Увеличиваем скорость левого мотора
      }
      getSpeedMotors(50,curSpeedL, curSpeedR);
      moveLeftMotorSpeed(leftMotorSpeed, curSpeedL);
      moveRightMotorSpeed(rightMotorSpeed, curSpeedR);
    }
    stop();
}
//Move forward for seconds
void lineRobot::moveBackwardSeconds(int seconds){
  resetEncoders();
  resetRegulators();
    long startTime = millis();
    unsigned int period = seconds*1000;
    float curSpeedL = 0;
    float curSpeedR = 0;
    while (period>millis()-startTime){ 
      float angleLeft = encoderRadianLeft();
      float angleRight = encoderRadianRight();
      int leftMotorSpeed = STANDARD_SPEED_PERCENTAGE;
      int rightMotorSpeed = STANDARD_SPEED_PERCENTAGE;
      // moveLeftMotor прямолинейности
      if (millis()-startTime<1000){
        float power = (millis()-startTime)/1000.0;
        leftMotorSpeed = leftMotorSpeed*constrain(power, 0.4, 1.0);
        rightMotorSpeed = rightMotorSpeed*constrain(power, 0.4, 1.0);
      }
      if (abs(angleLeft)>abs(angleRight)) {
          rightMotorSpeed += (angleLeft - angleRight) * kStraight; // Увеличиваем скорость правого мотора
      } else if (abs(angleLeft)<abs(angleRight)) {
          leftMotorSpeed += (angleRight - angleLeft) * kStraight; // Увеличиваем скорость левого мотора
      }
      getSpeedMotors(50,curSpeedL, curSpeedR);
      moveLeftMotorSpeed(-leftMotorSpeed, curSpeedL);
      moveRightMotorSpeed(-rightMotorSpeed, curSpeedR);
    }
    stop();
  //resetEncoders();
}
//retrun
int lineRobot::encoderDegreesRight(){
  return encoderPositionRight*360/pulsesPerRevolution;
}

int lineRobot::encoderDegreesLeft(){
  return encoderPositionLeft*360/pulsesPerRevolution;
}

float lineRobot::encoderRadianRight(){
  return encoderPositionRight*2.0*PI/pulsesPerRevolution;
}

float lineRobot::encoderRadianLeft(){
  return encoderPositionLeft*2.0*PI/pulsesPerRevolution;
}


void lineRobot::turnLeftAngle(int ang){
  resetEncoders();
  resetRegulators();
  float error = 0.1;
  targetAngle = ang*distance_between_wheel_and_center*PI/(RADIUS_WHEEL*180);
  long startTime = millis();
  unsigned int period = 40*ang+5000;
  //long ang_goal = get_angle_for_rotate(ang); 
  float angleLeft = 0.0;
  float angleRight = 0.0;
  lastTimeLeft = millis();
  lastTimeRight = millis();
  previousErrAngLeft = targetAngle;
  previousErrAngRight = targetAngle;
  float curSpeedL = 0;
  float curSpeedR = 0;
  while ((abs(previousErrAngLeft)>error || abs(previousErrAngRight)>error) && (period>millis()-startTime)){
    angleLeft = abs(encoderRadianLeft());
    angleRight = abs(encoderRadianRight());
    
      int leftMotorSpeed = computePidAngleMotor(targetAngle-angleLeft, kpAng, kdAng, kiAng,integralAngLeft,previousErrAngLeft, lastTimeLeft);
    int rightMotorSpeed = computePidAngleMotor(targetAngle-angleRight, kpAng, kdAng, kiAng,integralAngRight,previousErrAngRight, lastTimeRight);
   // Ограничение скорости в диапазоне от -100 до 100
    leftMotorSpeed = constrain(leftMotorSpeed, -75, 75);
    rightMotorSpeed = constrain(rightMotorSpeed, -75, 75);
    // плавный старт
    if (millis()-startTime<1000){
        float power = (millis()-startTime)/1000.0;
        leftMotorSpeed = leftMotorSpeed*constrain(power, 0.4, 1.0);
        rightMotorSpeed = rightMotorSpeed*constrain(power, 0.4, 1.0);
      }
    // moveLeftMotor прямолинейности
      if (abs(previousErrAngLeft)<abs(previousErrAngRight)) {
          rightMotorSpeed += (angleLeft - angleRight) * kStraight; // Увеличиваем скорость правого мотора
      } else if (abs(previousErrAngLeft)>abs(previousErrAngRight)) {
          leftMotorSpeed += (angleRight - angleLeft) * kStraight; // Увеличиваем скорость левого мотора
      }
    // Движение моторов с вычисленными скоростями
    getSpeedMotors(50,curSpeedL, curSpeedR);
    moveLeftMotorSpeed(-leftMotorSpeed*STANDARD_SPEED_PERCENTAGE/100, curSpeedL);
    moveRightMotorSpeed(rightMotorSpeed*STANDARD_SPEED_PERCENTAGE/100, curSpeedR);
  } 
  stop();
  //resetEncoders();
  delay(500);
}
void lineRobot::turnRightAngle(int ang){
  resetEncoders();
  resetRegulators();
  float error = 0.1;
  targetAngle = ang*distance_between_wheel_and_center*PI/(RADIUS_WHEEL*180);
  long startTime = millis();
  unsigned int period = 40*ang+5000;
  //long ang_goal = get_angle_for_rotate(ang); 
  float angleLeft = 0.0;
  float angleRight = 0.0;
  lastTimeLeft = millis();
  lastTimeRight = millis();
  previousErrAngLeft = targetAngle;
  previousErrAngRight = targetAngle;
  float curSpeedL = 0;
  float curSpeedR = 0;
  while ((abs(previousErrAngLeft)>error || abs(previousErrAngRight)>error) && (period>millis()-startTime)){
    angleLeft = abs(encoderRadianLeft());
    angleRight = abs(encoderRadianRight());
    int leftMotorSpeed = computePidAngleMotor(targetAngle-angleLeft, kpAng, kdAng, kiAng,integralAngLeft,previousErrAngLeft, lastTimeLeft);
    int rightMotorSpeed = computePidAngleMotor(targetAngle-angleRight, kpAng, kdAng, kiAng,integralAngRight,previousErrAngRight, lastTimeRight);

    // Ограничение скорости в диапазоне от -100 до 100
    leftMotorSpeed = constrain(leftMotorSpeed, -75, 75);
    rightMotorSpeed = constrain(rightMotorSpeed, -75, 75);
    // плавный старт
    if (millis()-startTime<1000){
        float power = (millis()-startTime)/1000.0;
        leftMotorSpeed = leftMotorSpeed*constrain(power, 0.4, 1.0);
        rightMotorSpeed = rightMotorSpeed*constrain(power, 0.4, 1.0);
      }
    // moveLeftMotor прямолинейности
      if (abs(previousErrAngLeft)<abs(previousErrAngRight)) {
          rightMotorSpeed += (angleLeft - angleRight) * kStraight; // Увеличиваем скорость правого мотора
      } else if (abs(previousErrAngLeft)>abs(previousErrAngRight)) {
          leftMotorSpeed += (angleRight - angleLeft) * kStraight; // Увеличиваем скорость левого мотора
      }
    // Движение моторов с вычисленными скоростями
    getSpeedMotors(50,curSpeedL, curSpeedR);
    moveLeftMotorSpeed(leftMotorSpeed*STANDARD_SPEED_PERCENTAGE/100, curSpeedL);
    moveRightMotorSpeed(-rightMotorSpeed*STANDARD_SPEED_PERCENTAGE/100, curSpeedR);
  } 
  stop();
  //resetEncoders();
  delay(500);
}
void lineRobot::resetLeftEncoder(){
  lineRobot::encoderPositionLeft = 0;
  //encLeft.clearCount();
}
void lineRobot::resetRightEncoder(){
  lineRobot::encoderPositionRight = 0;
  //encRight.clearCount();
}
void lineRobot::resetLeftEncoderValue(int value){
  encoderPositionLeft = value;
  //encLeft.setCount(value);
  //encLeft.clearCount();
}
void lineRobot::resetRightEncoderValue(int value){
  encoderPositionRight = value;
  //encRight.setCount(value);
  //encRight.clearCount();
}
void lineRobot::resetRegulators(){
    lastTimeLeftSpeed=millis()+3;
    lastTimeRightSpeed=millis()+3;
    lastTimeLeft=millis()+2;
    lastTimeRight=millis()+2;
    integralSpeedLeft=0;
    previousErrSpeedLeft=0;
    integralSpeedRight=0;
    previousErrSpeedRight=0;
    integralAngLeft=0;
    previousErrAngLeft=0;
    integralAngRight=0;
    previousErrAngRight=0;
}
void lineRobot::resetEncoders(){
  // long delta = abs(encLeft.getCount())-abs(encRight.getCount());
  // if (delta>15){
  //   resetLeftEncoderValue(delta);
  //   resetRightEncoder();
  //   return;
  // }
  // if (delta<-15){
  //   resetRightEncoderValue(delta);
  //   resetLeftEncoder();
  //   return;
  // }
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


lineRobot robot = lineRobot(4, 16, 12, 2,14,27,18,19, 3.3, 18.4, 2480);