#include <Stepper.h>
#include <MPU6050.h>

// Stepper motor setup
#define STEPS_PER_REV 2048  // Full-step mode (or 4096 for half-step)
Stepper stepperMotor(STEPS_PER_REV, A0, A2,A1, A3);  // IN1, IN3, IN2, IN4

// Tracking rotation
long stepCount = 0;  // Tracks total steps
float currentAngle = 0;  // Tracks current angle in degrees
bool stopRequested = false;  // Flag to track stop request
float stopAngle = 0;
#define LED_PIN 8
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
int zeroCount = 0;
// Variables to store encoder counts
volatile long leftPulses = 0;
volatile long rightPulses = 0;
String command = ""; // Buffer to store incoming characters
const float wheelCircumference = 21.99; //centimeters
const int pulsesPerRevolution = 20; //# of holes in encoder wheel
const float wheelBase = 18.73;  //distance between wheels in cm
const float wheelDiameter = 7.01;
int manualFlag = 0;
int testCount = 0;
float angleIn = 0;
MPU6050 mpu;
float angleZ = 0; // Current yaw angle
unsigned long prevTime;
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

enum Car{idle, scan, rotate, drive, manual, finalPosition}state;

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
      if ((laserDetect != 1) && (laserDetect != 2)) {
        state = scan;
      }
      else if (laserDetect == 1) {
        stopRequested = true;
        digitalWrite(LED_PIN, HIGH);
        stopAngle = currentAngle;
        if (stopAngle > 180) {
          stopAngle = stopAngle - 360;
        }
      }
      else if (laserDetect == 2) {
        stopRequested = true;
        manualFlag = 1;
      }
      if (stopRequested && (stepCount % 64 == 0)) {
        stepperMotor.step(0);
        if (manualFlag == 0) {
          state = rotate;
        }
        else if (manualFlag == 1) {
          state = manual;
        }
      }   
      
    break;
    case rotate:
      state = drive;
    break;
    case drive:
      if (laserDetect != 0 || zeroCount <= 2000) {
        state = drive;
      }
      else if (laserDetect == 0 && zeroCount > 2000) {
        zeroCount = 0;
        currentAngle = 0;
        state = finalPosition;
      }
    break;
    case manual:
      if (laserDetect == 2) {
        manualFlag = 0;
        state = scan;
      }
      else if (laserDetect != 2) {
        state = manual;
      }
    break;
    case finalPosition:
      stopRequested = false;
      digitalWrite(LED_PIN, LOW);
      state = scan;
    break;
  }

  switch(state) {   //Actions
    case idle:
      //laserDetect = 0;
      Serial.println("Idle");
    break;
    case scan:
      cameraSpin();
      // testCount++;
      // if (testCount > 3) {
      //   laserDetect = 1;
      //   testCount = 0;
      // }
      Serial.println("Scan");
    break;
    case rotate:
      Serial.println(stopAngle);
      rotateTo(stopAngle);
      //delay(3000);
      Serial.println("Rotate");
    break;
    case drive:
      processCommandAuto(laserDetect);
      Serial.println(laserDetect);
      // testCount++;
      // delay(500);
      // if (testCount > 3) {
      //   laserDetect = 0;
      //   testCount = 0;
      //   zeroCount = 4;
      // }
      if (laserDetect == 0) {
        zeroCount++;  //counting 5 ticks without laser detected to stop Drive
      }
      else if (laserDetect != 0) {
        zeroCount = 0;
      }
      Serial.println(zeroCount);
      Serial.println("Drive");
      //Serial.println(testCount);
    break;
    case manual:
      processCommandManual(laserDetect);
      Serial.println("manual");
      Serial.println(laserDetect);
    break;
    case finalPosition:
      Serial.println("Final Position"); 
      goStraight(80);
      delay(800);
      stopMotors();
      //delay(500);
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
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), countEncoder2, RISING);
  stepperMotor.setSpeed(10);  // Set speed to 10 RPM
  Wire.begin();
  mpu.initialize();
    
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  prevTime = millis();
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

void processCommandAuto(int camera) {
  if (camera == 0) {
    stopMotors();
  }
  else if (camera != 0) {
    // Go straight
    goStraight(80);
  }
}

void processCommandManual(int input) {
  if (input == 0) {
    stopMotors();
  }
  else if (input == 10) {
    rotateTo(10);
  }
  else if (input == -10) {
    rotateTo(-10);
  }
  else if (input == 1) {
    goStraight(80);
  }
}

void rotateTo(float targetAngle) {
    int16_t gx, gy, gz;
    float gyroZ, dt;
    unsigned long currentTime;

    angleZ = 0; // Reset current angle to 0 before starting rotation
    prevTime = millis();

    while (abs(angleZ) < (abs(targetAngle) - 10)) {
        // Read gyroscope data
        mpu.getRotation(&gx, &gy, &gz);
        gyroZ = gz / 131.0; // Convert raw data to degrees/sec

        // Compute elapsed time
        currentTime = millis();
        dt = (currentTime - prevTime) / 1000.0;
        prevTime = currentTime;

        // Integrate gyroZ to get angle
        angleZ += gyroZ * dt;

        // Rotate motors based on sign of target angle
        if (targetAngle > 0) {
          // Set right rotation motor speeds here
          Serial.println("Turning Right");
          digitalWrite(in1, LOW); 
          digitalWrite(in2, HIGH);
          analogWrite(enA, 80);
          digitalWrite(in3, HIGH); 
          digitalWrite(in4, LOW);
          analogWrite(enB, 94);
        } 
        else {
            Serial.println("Turning Left");
            // Set left rotation motor speeds here
            digitalWrite(in1, HIGH); 
            digitalWrite(in2, LOW);
            analogWrite(enA, 80);
            digitalWrite(in3, LOW); 
            digitalWrite(in4, HIGH);
            analogWrite(enB, 94);
        }
    }
    // Stop motors when angle is reached
    stopMotors();
    Serial.println("Angle Reached");

    // Reset the reference point for future rotations
    angleZ = 0;
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