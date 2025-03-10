#include <PID_v1.h>

// Right Motor connections
int enA = 11;
int in1 = 10;
int in2 = 9;
// Left Motor connections
int enB = 6;
int in3 = 4;
int in4 = 5;
int encoder1 = 2; // Left motor encoder
int encoder2 = 3; // Right motor encoder

// Variables to store encoder counts
volatile long leftPulses = 0;
volatile long rightPulses = 0;

// PID variables
double setpointLeft, inputLeft, outputLeft;
double KpLeft = 0.5, KiLeft = 0.1, KdLeft = 0.1;
double setpointRight, inputRight, outputRight;
double KpRight = 0.5, KiRight = 0.1, KdRight = 0.1;

PID myPIDLeft(&inputLeft, &outputLeft, &setpointLeft, KpLeft, KiLeft, KdLeft, DIRECT);
PID myPIDRight(&inputRight, &outputRight, &setpointRight, KpRight, KiRight, KdRight, DIRECT);

// Interrupt service routines
void countEncoder1() {
  leftPulses++;
}
void countEncoder2() {
  rightPulses++;
}

void setup() {
  // Set motor driver pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoder1), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), countEncoder2, RISING);

  // Set up PID
  myPIDLeft.SetMode(AUTOMATIC);
  myPIDLeft.SetOutputLimits(0, 255);

  myPIDRight.SetMode(AUTOMATIC);
  myPIDRight.SetOutputLimits(0, 255);

  Serial.begin(115200);
}

void loop() {
  // Set desired speed
  setpointLeft = 80;
  setpointRight = 80;

  // Get current speed
  inputLeft = getSpeed(leftPulses);
  inputRight = getSpeed(rightPulses);

  // Compute PID output
  myPIDLeft.Compute();
  myPIDRight.Compute();
  
  // Update motor speed
  analogWrite(enA, outputLeft);
  analogWrite(enB, outputRight);

  delay(100); // Adjust delay as needed
}

double getSpeed(volatile long &encoderPulses) {
  static unsigned long lastTime = millis();
  static long lastPulses = 0;
  unsigned long currentTime = millis();
  double elapsedTime = (currentTime - lastTime) / 1000.0; // Convert ms to sec

  double speed = (encoderPulses - lastPulses) / elapsedTime; // Pulses per second
  lastPulses = encoderPulses;  // Store current pulse count for next calculation
  lastTime = currentTime;

  return speed;
}