#include <Stepper.h>

// Stepper motor setup
#define STEPS_PER_REV 2048  // Full-step mode (or 4096 for half-step)
Stepper stepperMotor(STEPS_PER_REV, A0, A2,A1, A3);  // IN1, IN3, IN2, IN4

// Tracking rotation
long stepCount = 0;  // Tracks total steps
float currentAngle = 0;  // Tracks current angle in degrees
bool stopRequested = false;  // Flag to track stop request
float stopAngle = 0;
void setup() {
    Serial.begin(9600);
    stepperMotor.setSpeed(10);  // Set speed to 10 RPM
}

void loop() {
    int stepsToMove = 32;  // Example: move 100 steps
    stepperMotor.step(stepsToMove);

    // Update step count and calculate angle
    stepCount++;
    currentAngle = stepCount * 5.625; //11.25 is the angle per step (32 steps)

    // Print rotation info
    Serial.print("Total Steps: ");
    Serial.println(stepCount);
    Serial.print(" | Angle: ");
    Serial.println(currentAngle);
  if (stepCount >= 64) {
    stepCount = 0;
    currentAngle = 0;
  }

   // Check for user input to stop
    if (Serial.available()) {
        Serial.read();  // Read and discard input
        stopRequested = true;
        stopAngle = currentAngle;
        Serial.println("Stop requested! Completing full rotation...");

    }

    // If stop requested, complete the current revolution
    if (stopRequested && (stepCount % 64 == 0)) {
        Serial.print("Final Angle: ");
        Serial.println(stopAngle);
        Serial.println("Rotation complete. Stopping motor.");
        while (true);  // Halt execution
    }

  
    //delay(500);  // Small delay before next move (for testing)
}

