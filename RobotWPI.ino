#include <Arduino.h>
#include <Chassis.h>
#include <Timer.h>
#include <Romi32U4.h>

// Create an instance of the Chassis class
const float WHEEL_DIAMETER = 7.0;       // cm
const float TICKS_PER_REV = 1440.0;     // ticks per wheel revolution
const float WHEEL_TRACK = 14.7;         // cm (distance between wheels)

Chassis chassis(WHEEL_DIAMETER, TICKS_PER_REV, WHEEL_TRACK);

// Constants for trapezoidal motion profile
const float MAX_LINEAR_SPEED = 20.0;    // cm/s
const float MAX_LINEAR_ACCEL = 50.0;    // cm/s^2
const float MAX_ANGULAR_SPEED = 90.0;   // degrees/s
const float MAX_ANGULAR_ACCEL = 180.0;  // degrees/s^2

// Define the structure for a path segment
enum SegmentType { MOVE_STRAIGHT, TURN };

struct PathSegment {
  SegmentType type;
  float value; // distance in cm for straight, angle in degrees for turn
};

// Forwards
  // {MOVE_STRAIGHT, 50.0},
  // {MOVE_STRAIGHT, 100.0},
  // {MOVE_STRAIGHT, 150.0},
  // {MOVE_STRAIGHT, 200.0},
// Reverses
  // {MOVE_STRAIGHT, -50.0},
  // {MOVE_STRAIGHT, -100.0},
  // {MOVE_STRAIGHT, -150.0},
  // {MOVE_STRAIGHT, -200.0},
// For. Into Sq. & Turn L
  // {MOVE_STRAIGHT, 34.0},
  // {TURN, -86.0},
  // {MOVE_STRAIGHT, 16.0},
// For. Into Sq. & Turn R
  // {MOVE_STRAIGHT, 34.0},
  // {TURN, 90.0},
  // {MOVE_STRAIGHT, 16.0},
// Rev. Into Sq. & Turn L
  // {MOVE_STRAIGHT, -16.0},
  // {TURN, -90.0},
  // {MOVE_STRAIGHT, 16.0},
// Rev. Into Sq. & Turn R
  // {MOVE_STRAIGHT, -16.0},
  // {TURN, 90.0},
  // {MOVE_STRAIGHT, 16.0},
// Into gate zone and rev out
  // {MOVE_STRAIGHT, 34.0},
  // {MOVE_STRAIGHT, -34.0},
// Into end square
  // {MOVE_STRAIGHT, 34.0},

// Define the path as an array of PathSegments
PathSegment path[] = {
  {MOVE_STRAIGHT, 50.0},
  // Add more movements during competition
};

const int numSegments = sizeof(path) / sizeof(PathSegment);

// Target time in seconds
const float TARGET_TIME = 40.0; // seconds

// Variables for path execution
int currentSegment = 0;
float baseSpeed = 0.0;

// Timer for controlling loop intervals
Timer controlTimer(10); // Initialize with a 10 ms interval

// Variables for calculations
float cmPerTick = 0.0;
float wheelBase = WHEEL_TRACK; // cm

// Button A
Romi32U4ButtonA buttonA;
bool isRunning = false;

// Motion variables for executeStraight()
bool firstCallStraight = true;
float distanceTraveledStraight = 0.0;
float speed = 0.0;

// Motion variables for executeTurn()
bool firstCallTurn = true;
float angleTurned = 0.0;
float angularSpeed = 0.0;

void setup() {
  Serial.begin(9600);
  chassis.init();
  chassis.setMotorPIDcoeffs(5.0, 0.5); // Adjust PID coefficients as needed

  // Calculate cm per encoder tick
  float wheelCircumference = PI * WHEEL_DIAMETER; // cm
  cmPerTick = wheelCircumference / TICKS_PER_REV; // cm per tick

  // Calculate total path distance and total turn angle
  float totalPathDistance = 0.0;
  float totalTurnAngle = 0.0;
  for (int i = 0; i < numSegments; i++) {
    if (path[i].type == MOVE_STRAIGHT) {
      totalPathDistance += abs(path[i].value);
    } else if (path[i].type == TURN) {
      totalTurnAngle += abs(path[i].value);
    }
  }

  // Estimate time required for turns
  float totalTurnTime = 0.0;
  for (int i = 0; i < numSegments; i++) {
    if (path[i].type == TURN) {
      float angle = abs(path[i].value);

      // Time to accelerate to max angular speed
      float angularAccelTime = MAX_ANGULAR_SPEED / MAX_ANGULAR_ACCEL;
      // Angle covered during acceleration
      float angularAccelAngle = 0.5 * MAX_ANGULAR_ACCEL * angularAccelTime * angularAccelTime;

      float timePerTurn = 0.0;
      if (angle < 2 * angularAccelAngle) {
        // Cannot reach max angular speed
        float accelTime = sqrt(angle / MAX_ANGULAR_ACCEL);
        timePerTurn = 2 * accelTime;
      } else {
        float constantAngularSpeedAngle = angle - 2 * angularAccelAngle;
        float constantAngularSpeedTime = constantAngularSpeedAngle / MAX_ANGULAR_SPEED;
        timePerTurn = 2 * angularAccelTime + constantAngularSpeedTime;
      }
      totalTurnTime += timePerTurn;
    }
  }

  // Calculate available time for straight movements
  float availableStraightTime = TARGET_TIME - totalTurnTime;
  if (availableStraightTime <= 0) {
    availableStraightTime = TARGET_TIME * 0.8; // Allocate at least some time for straight movement
    totalTurnTime = TARGET_TIME - availableStraightTime;
  }

  // Calculate base speed to meet the available time for straights
  baseSpeed = totalPathDistance / availableStraightTime;

  // Limit base speed to MAX_LINEAR_SPEED
  if (baseSpeed > MAX_LINEAR_SPEED) {
    baseSpeed = MAX_LINEAR_SPEED;
  }

  // For debugging
  Serial.print("Total straight distance: ");
  Serial.print(totalPathDistance);
  Serial.println(" cm");
  Serial.print("Total turn angle: ");
  Serial.print(totalTurnAngle);
  Serial.println(" degrees");
  Serial.print("Total turn time: ");
  Serial.print(totalTurnTime);
  Serial.println(" s");
  Serial.print("Available straight time: ");
  Serial.print(availableStraightTime);
  Serial.println(" s");
  Serial.print("Calculated base speed: ");
  Serial.print(baseSpeed);
  Serial.println(" cm/s");

  Serial.println("Press Button A to start the robot.");
}

void loop() {
  // Check if Button A is pressed to start/stop the robot
  if (buttonA.getSingleDebouncedRelease()) {
    isRunning = !isRunning; // Toggle the running state
    if (isRunning) {
      Serial.println("Robot started.");
      // Reset variables to start from the beginning
      currentSegment = 0;
      resetMotionVariables();
    } else {
      Serial.println("Robot stopped.");
      // Stop the robot
      chassis.setWheelSpeeds(0, 0);
    }
  }

  if (isRunning && controlTimer.isExpired()) { // Control loop every 10 ms
    executePath();
  }
}

void executePath() {
  if (currentSegment >= numSegments) {
    // Path complete, stop the robot
    chassis.setWheelSpeeds(0, 0);
    Serial.println("Path complete.");
    isRunning = false;
    return;
  }

  PathSegment segment = path[currentSegment];

  if (segment.type == MOVE_STRAIGHT) {
    executeStraight(segment.value);
  } else if (segment.type == TURN) {
    executeTurn(segment.value);
  }
}

void resetMotionVariables() {
  // Reset motion variables for executeStraight()
  firstCallStraight = true;
  distanceTraveledStraight = 0.0;
  speed = 0.0;

  // Reset motion variables for executeTurn()
  firstCallTurn = true;
  angleTurned = 0.0;
  angularSpeed = 0.0;

  // Reset encoders
  chassis.leftMotor.getAndResetCount();
  chassis.rightMotor.getAndResetCount();
}

void executeStraight(float distance) {
  if (firstCallStraight) {
    firstCallStraight = false;
    speed = 0.0;
    distanceTraveledStraight = 0.0;

    // Reset encoders
    chassis.leftMotor.getAndResetCount();
    chassis.rightMotor.getAndResetCount();

    Serial.print("Starting straight segment of ");
    Serial.print(distance);
    Serial.println(" cm.");
  }

  // Get current average encoder count
  long deltaLeft = chassis.leftMotor.getCount();
  long deltaRight = chassis.rightMotor.getCount();
  float distanceLeft = deltaLeft * cmPerTick;
  float distanceRight = deltaRight * cmPerTick;
  distanceTraveledStraight = (distanceLeft + distanceRight) / 2.0;

  // Determine direction
  float direction = (distance >= 0) ? 1.0 : -1.0;

  // Calculate the absolute distances
  float distanceToTravel = fabs(distance);
  float distanceTraveled = fabs(distanceTraveledStraight);

  float remainingDistance = distanceToTravel - distanceTraveled;
  float decelDistance = (speed * speed) / (2 * MAX_LINEAR_ACCEL);

  if (remainingDistance <= decelDistance) {
    speed -= MAX_LINEAR_ACCEL * 0.01; // 0.01 seconds per control loop
    if (speed < 0) speed = 0;
  } else if (speed < baseSpeed) {
    speed += MAX_LINEAR_ACCEL * 0.01;
    if (speed > baseSpeed) speed = baseSpeed;
  }

  // Apply direction to speed
  float wheelSpeed = direction * speed;

  // Set wheel speeds
  chassis.setWheelSpeeds(wheelSpeed, wheelSpeed);

  // Print status
  Serial.print("Distance traveled: ");
  Serial.print(distanceTraveledStraight);
  Serial.print(" cm, Speed: ");
  Serial.print(wheelSpeed);
  Serial.println(" cm/s");

  // Check if segment is complete
  if (distanceTraveled >= distanceToTravel) {
    chassis.setWheelSpeeds(0, 0);
    firstCallStraight = true;
    speed = 0.0; // Reset speed for next segment
    currentSegment++;
    Serial.println("Straight segment complete.");
  }
}

void executeTurn(float angle) {
  if (firstCallTurn) {
    firstCallTurn = false;
    angularSpeed = 0.0;
    angleTurned = 0.0;

    // Reset encoders
    chassis.leftMotor.getAndResetCount();
    chassis.rightMotor.getAndResetCount();

    Serial.print("Starting turn of ");
    Serial.print(angle);
    Serial.println(" degrees.");
  }

  // Calculate angle turned based on wheel encoder differences
  long deltaLeft = chassis.leftMotor.getCount();
  long deltaRight = chassis.rightMotor.getCount();
  float distanceLeft = deltaLeft * cmPerTick;
  float distanceRight = deltaRight * cmPerTick;
  float rotation = (distanceLeft - distanceRight) / wheelBase; // radians
  angleTurned = rotation * (180.0 / PI); // degrees

  // Determine target angular speed using trapezoidal profile
  float remainingAngle = angle - angleTurned;
  float decelAngle = (angularSpeed * angularSpeed) / (2 * MAX_ANGULAR_ACCEL);
  if (fabs(remainingAngle) <= decelAngle) {
    angularSpeed -= MAX_ANGULAR_ACCEL * 0.01; // 0.01 seconds per control loop
    if (angularSpeed < 0) angularSpeed = 0;
  } else if (angularSpeed < MAX_ANGULAR_SPEED) {
    angularSpeed += MAX_ANGULAR_ACCEL * 0.01;
    if (angularSpeed > MAX_ANGULAR_SPEED) angularSpeed = MAX_ANGULAR_SPEED;
  }

  // Set direction based on turn direction
  float turnDirection = (angle >= 0) ? 1.0 : -1.0;

  // Convert angular speed to linear wheel speed
  float wheelSpeed = (angularSpeed * PI / 180.0) * (wheelBase / 2.0); // cm/s
  float leftSpeed = turnDirection * wheelSpeed;
  float rightSpeed = -turnDirection * wheelSpeed;
  chassis.setWheelSpeeds(leftSpeed, rightSpeed);

  // Print status
  Serial.print("Angle turned: ");
  Serial.print(angleTurned);
  Serial.print(" degrees, Angular Speed: ");
  Serial.print(angularSpeed);
  Serial.println(" deg/s");

  // Check if segment is complete
  if (fabs(angleTurned) >= fabs(angle)) {
    chassis.setWheelSpeeds(0, 0);
    firstCallTurn = true;
    angularSpeed = 0.0; // Reset angular speed for next segment
    currentSegment++;
    Serial.println("Turn segment complete.");
  }
}
