#include <lineRobot.h>
// Robot connections
// Left motor: in1 - 2, in2 - 4. Right motor: in3 - 12, in4 -13
// Left encoder: 16, 17. Right encoder: 18, 19

void setup() {
  // Robot rotation for 45 degress to the left
  robot.turnLeftAngle(45);
  // Robot rotation for 30 degress to the right
  robot.turnRightAngle(30);
  // Robot rotation for 90 degress to the left
  robot.turnRight();
  // Robot rotation for 90 degress to the right
  robot.turnRight();
}

void loop(){
}