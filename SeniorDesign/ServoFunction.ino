#include <Servo.h>

Servo xServo;  // create servo object to control a servo
Servo yServo;

// distances done in meters HARD CODED AT THE MOMENT BUT SHOULD BE INPUTED IN FINAL DESIGN
double height = 0.16;
double xDistance = 0;
double yDistance = 0;

int xPos = 90;    // variable to store the xServo position (90 is the neutral position for x)
int yPos = 90;    // variable to store the yServo position (90 is the neutral position for y)

// function to receive input values NOT DONE YET
void recieveDistanceValues();

// functions to calculate position of servo given our coordinates
int calculateXPos(double xDistance);
int calculateYPos(double yDistance);

// function to calculate theta angle to set the servo
double calculateAngle(double axisDistance);

// conversion functions
double radiansToDegrees(double radians);
double degreesToRadians(double degrees);

void setup() {
  xServo.attach(9);  // attaches the x servo on pin 9 
  yServo.attach(10); // attaches the y servo to pin 10
  Serial.begin(9600);
}

void loop() {
  // ERROR CHECK FOR NOW TO PREVENT NEGATIVE Y VALUES
  if (yDistance < 0) {
    Serial.println("Error: Negative yDistance is not allowed."); // Log the error
    yDistance = 0;  // Reset yDistance to a valid value
  }

  // recieveDistanceValues(); NEED THIS ADDED EVENTUALLY


  // calculate servo positions given the coordinates
  xPos = calculateXPos(xDistance);
  yPos = calculateYPos(yDistance);

  // // move servos to where they need to be
  xServo.write(xPos);
  yServo.write(yPos);

  delay(15);
}

// receive distance input
void receiveDistanceValues() {
}

// x postion calculation
int calculateXPos(double xDistance) {
  if (xDistance < 0) {
    xPos = 90 - calculateAngle(-xDistance);
  }
  else if (xDistance > 0) {
    xPos = 90 + calculateAngle(xDistance);
  }
  else {
    xPos = 90;
  }
  xPos = constrain(xPos, 0, 180); //ensures that the value is within the accepted range
  return xPos;
}

// y position calculation
int calculateYPos(double yDistance) {
  if (yDistance == 0) {
    yPos = 90;
  }
  else {
    yPos = 90 + calculateAngle(yDistance);
  }
  yPos = constrain(yPos , 90 , 180); // ensure we are within accepted range of servo boundaries
  return yPos;
}

// given a distance value, calculate angle
double calculateAngle(double axisDistance) {
  double theta = radiansToDegrees((atan(axisDistance / height)));
  return theta;
}

// conversion functions
double degreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}
double radiansToDegrees(double radians) {
  return radians * 180.0 / M_PI;
}
