#include <lineRobot.h>
// Robot connections
// Left motor: in1 - 2, in2 - 4. Right motor: in3 - 12, in4 -13
// Left encoder: 16, 17. Right encoder: 18, 19

void setup() {
  // Motion forward with 50% of speed for 20 centimiters
  robot.moveForwardSpeedDistance(50, 20);
}

void loop(){
}