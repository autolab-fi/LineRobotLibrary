#include <lineRobot.h>
// Robot connections
// Left motor: in1 - 4, in2 - 6. Right motor: in3 - 8, in4 -10
// Left encoder: 16, 17. Right encoder: 18, 19
// Robot parameters: radius of wheels - 3 cm. Distance between wheels - 20 cm.
lineRobot myRobot = lineRobot(4, 6, 8, 10, 3, 20);
void setup() {
  // Motion forward with 50% of speed for 20 centimiters
  myRobot.moveForwardSpeedDistance(70, 30);
  // Motion backward for 30 centimiters with minimal speed
  myRobot.moveBackwardDistance(30);
}

void loop(){
}