#include <Stepper.h>

// Stepper motor setup
#define STEPS_PER_REV 2048  // Full-step mode (or 4096 for half-step)
Stepper stepperMotor(STEPS_PER_REV, A0, A2,A1, A3);  // IN1, IN3, IN2, IN4

// Tracking rotation
long stepCount = 0;  // Tracks total steps
float currentAngle = 0;  // Tracks current angle in degrees
bool stopRequested = false;  // Flag to track stop request
float stopAngle = 0;
// Right Motor connections
int enA = 11;
int in1 = 10;
int in2 = 9;
// Left Motor connections
int enB = 6;
int in3 = 4;
int in4 = 5;
int laserDetect = -1;
const float Pi = 3.14159;
unsigned long lastDebounceTimeA = 0;
unsigned long lastDebounceTimeB = 0;
const unsigned long debounceDelay = 1000;
int encoder1 = 2; //Left motor encoder
int encoder2 = 3; //Right motor encoder

// Variables to store encoder counts
volatile long leftPulses = 0;
volatile long rightPulses = 0;
String command = ""; // Buffer to store incoming characters
const float wheelCircumference = 21.99; //centimeters
const int pulsesPerRevolution = 20; //# of holes in encoder wheel
const float wheelBase = 18.73;  //distance between wheels in cm
const float wheelDiameter = 7.01;
float angleIn = 0;
// Interrupt service routines
void countEncoder1() {
  unsigned long currentTime = micros();
  if ((currentTime - lastDebounceTimeA) > debounceDelay) {
    leftPulses++;
    lastDebounceTimeA = currentTime;
  }
}
void countEncoder2() {
  unsigned long currentTime = micros();
  if ((currentTime - lastDebounceTimeB) > debounceDelay) {
    rightPulses++;
    lastDebounceTimeB = currentTime;
  }
}

enum Car{idle, scan, rotate, drive, finalPosition}state;

void tick() {
  switch(state) {   //Transitions
    case idle:
      if (laserDetect != 0) {
        state = idle;
      }
      else if (laserDetect == 0) {
        state = scan;
      }
      break;
    case scan:
      if (laserDetect != 1) {
        state = scan;
      }
      else if (laserDetect == 1) {
        stopRequested = true;
        stopAngle = currentAngle;
      }
      if (stopRequested && (stepCount % 64 == 0)) {
          stepperMotor.step(0);
          state = rotate;
      }   
    break;
    case rotate:
      state = drive;
    break;
    case drive:
      if (laserDetect != 0) {
        state = drive;
      }
      else if (laserDetect == 0) {
        state = finalPosition;
      }
    break;
    case finalPosition:
      stopRequested = false;
      state = scan;
    break;
  }

  switch(state) {   //Actions
    case idle:
      //Serial.println("Idle");
    break;
    case scan:
      cameraSpin();
      //Serial.println("Scan");
    break;
    case rotate:
      Serial.println(stopAngle);
      rotateTo(stopAngle);
      //delay(3000);
      //Serial.println("Rotate");
    break;
    case drive:
      processCommand(command);
      //Serial.println("Drive");
    break;
    case finalPosition:
      //Serial.println("Final Position");
    break;
  }

}

void setup() {
  // put your setup code here, to run once:
  // Set motor driver pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), countEncoder2, RISING);
  stepperMotor.setSpeed(10);  // Set speed to 10 RPM
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()) {  // Check if data is available
    char c = Serial.read();   // Read one character
    if (c == '\n') {          // End of command
      laserDetect = command.toInt();
      Serial.println(laserDetect);
      command = "";         // Clear the buffer
    } 
    else {
      command += c;         // Append character to the buffer
    }
  }
  tick();

}

void processCommand(String cmd) {
  int camera = cmd.toInt();
  if (camera == 0) {
    stopMotors();
  }
  else if (camera == 10) {
    // Turn left
    rotateTo(10);
  } 
  else if (camera == -10) {
    // Turn right
    rotateTo(-10);
  } 
  else if (camera == 1) {
    // Go straight
    goStraight(80);
  }
}

void rotateTo(float angle) {
    leftPulses = 0;
    rightPulses = 0;
    
    float degreesPerPulse = (360 * wheelBase) / (Pi * wheelDiameter * pulsesPerRevolution);
    long targetPulses = (abs(angle) / degreesPerPulse) * Pi;
    if (targetPulses < 5) {
    targetPulses = 5; // Ensure a minimum movement
    }
    if (angle > 0) {  // Rotate Right
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW);
        analogWrite(enA, 100);
        digitalWrite(in3, HIGH); 
        digitalWrite(in4, LOW);
        analogWrite(enB, 100);
    } else {  // Rotate Left
        digitalWrite(in1, LOW); 
        digitalWrite(in2, HIGH);
        analogWrite(enA, 100);
        digitalWrite(in3, LOW); 
        digitalWrite(in4, HIGH);
        analogWrite(enB, 100);
    }

    while ((leftPulses < targetPulses) && (rightPulses < targetPulses)) {
    }

    digitalWrite(in1, LOW); 
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); 
    digitalWrite(in4, LOW);
}

void goStraight(int speed) {
  digitalWrite(in1, HIGH); // Left motor forward
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);

  digitalWrite(in3, LOW); // Right motor forward
  digitalWrite(in4, HIGH);
  analogWrite(enB, 90);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void cameraSpin() {
  int stepsToMove = 32;  // Example: move 100 steps
  stepperMotor.step(stepsToMove);
  // Update step count and calculate angle
  stepCount++;
  currentAngle = stepCount * 5.625; //11.25 is the angle per step (32 steps)
  // Print rotation info
  // Serial.print("Total Steps: ");
  // Serial.println(stepCount);
  // Serial.print(" | Angle: ");
  // Serial.println(currentAngle);
  if (stepCount >= 64) {
    stepCount = 0;
    currentAngle = 0;
  }
}