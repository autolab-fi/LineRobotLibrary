# Documentation

## Robot Initialization

### `lineRobot(int pinLeft1, int pinLeft2, int pinRight1, int pinRight2, float _radius_wheel, float distance_between_wheels)`
Constructor for creating a robot object. Returns an object of type `lineRobot`.
- `pinLeft1`: First pin of the left motor. When connected through a driver - in1.
- `pinLeft2`: Second pin of the left motor. When connected through a driver - in2.
- `pinRight1`: First pin of the right motor. When connected through a driver - in3.
- `pinRight2`: Second pin of the right motor. When connected through a driver - in4.
- `wheel_radius`: Radius of the wheels installed on the robot in centimeters.
- `distance_between_wheels`: Distance between the wheels of the robot in centimeters.

## Basic Motion Functions

### `startMotorForwardLeft(int sp)`
Function to start the left motor in the forward direction.
- `sp`: the speed of the motor. The argument value ranges from 0 to 100.

### `startMotorBackwardLeft(int sp)`
Function to start the left motor in the backward direction.
- `sp`: the speed of the motor. The argument value ranges from 0 to 100.

### `startMotorForwardRight(int sp)`
Function to start the right motor in the forward direction.
- `sp`: the speed of the motor. The argument value ranges from 0 to 100.

### `startMotorBackwardRight(int sp)`
Function to start the right motor in the backward direction.
- `sp`: the speed of the motor. The argument value ranges from 0 to 100.

### `stopMotorLeft()`
Function to stop the left motor.

### `stopMotorRight()`
Function to stop the right motor.

### `stopRobot()`
Function to stop both the right and left motors.

### `moveForwardSpeedDistance(int sp, float dist)`
Function to initiate the robot's forward motion for a specific distance at a given speed.
- `sp`: the speed of the robot. The argument value ranges from 0 to 100.
- `dist`: the distance for the robot's movement, specified in centimeters.

### `moveBackwardSpeedDistance(int sp, float dist)`
Function to initiate the robot's backward motion for a specific distance at a given speed.
- `sp`: the speed of the robot. The argument value ranges from 0 to 100.
- `dist`: the distance for the robot's movement, specified in centimeters.

### `moveForwardDistance(float dist)`
Function to initiate the robot's forward motion for a specific distance.
- `dist`: the distance for the robot's movement, specified in centimeters.

### `moveBackwardDistance(float dist)`
Function to initiate the robot's backward motion for a specific distance.
- `dist`: the distance for the robot's movement, specified in centimeters.

### `moveForwardSeconds(int seconds)`
Function to initiate the robot's forward motion for a specific number of seconds.
- `seconds`: the number of seconds during which the robot will move forward.

### `moveBackwardSeconds(int seconds)`
Function to initiate the robot's backward motion for a specific number of seconds.
- `seconds`: the number of seconds during which the robot will move backward.

### `moveMotors(int dir, int sp)`
Function to control motor speeds while moving in forward and backward directions.
Returns the distance traveled by the robot. Return type is long.
- `dir`: 0 - forward direction, 1 - backward direction.
- `sp`: the speed of the robot. The argument value ranges from 0 to 100.

### `turnLeftAngle(int ang)`
Function to turn the robot to the left by a specified angle.
- `ang`: the angle by which the robot should turn.

### `turnRightAngle(int ang)`
Function to turn the robot to the right by a specified angle.
- `ang`: the angle by which the robot should turn.

### `turnLeft()`
Function to turn the robot to the left by 90 degrees.

### `turnRight()`
Function to turn the robot to the right by 90 degrees.

### `rotate()`
Function to rotate the robot through the left shoulder.

## Working with the Encoder
### `getPositionLeftEncoder()`
Function to retrieve values from the left motor encoder in steps.
Returns a long data type.

### `getPositionRightEncoder()`
Function to retrieve values from the right motor encoder in steps.
Returns a long data type.

### `getPositionLeftEncoderDegrees()`
Function to retrieve values from the left motor encoder in degrees.
Returns a long data type.

### `getPositionRightEncoderDegrees()`
Function to retrieve values from the right motor encoder in degrees.
Returns a long data type.

### `resetLeftEncoder()`
Function to reset values from the left motor encoder.

### `resetRightEncoder()`
Function to reset values from the right motor encoder.

## Additional Functions
### `int computePID(float input, float setpoint, float kp, float kd, float ki, float dt, int minOut, int maxOut)`
PID controller. Returns a control signal. Returns an integer data type.
- `input`: Current value of the measured variable you want to control (e.g., the current position of the robot).

- `setpoint`: Desired value for the measured variable (e.g., the desired position of the robot).

- `kp`: Proportional gain coefficient - determines how strongly the system reacts to the current error (difference between measured and desired values).

- `ki`: Integral gain coefficient - determines how strongly the system reacts to accumulated errors over time.

- `kd`: Derivative gain coefficient - determines how strongly the system reacts to the rate of change of error.

- `dt`: Time between iterations in the system. This is used to calculate the discrete integral and derivative.

- `minOut`: Minimum value of the control signal.

- `maxOut`: Maximum value of the control signal.

### `setRotateCoefficient_kp(float value)`
Function to change the proportional component coefficient of the PID controller for robot rotation functions.
- `value`: new value. Default value is 0.001.

### `setRotateCoefficient_ki(float value)`
Function to change the integral component coefficient of the PID controller for robot rotation functions.
- `value`: new value. Default value is 0.0001.

### `setRotateCoefficient_kd(float value)`
Function to change the derivative component coefficient of the PID controller for robot rotation functions.
- `value`: new value. Default value is 0.00001.

### `setStraightMotionCoefficient(float value)`
Function to change the coefficient for aligning the speeds of the robot's motors.
- `value`: new value. Default value is 0.0001.

### `setEncoderDegrees(float value)`
Function to change the coefficient for converting encoder readings to degrees.
- `value`: new coefficient value. Default value is 2.65.
