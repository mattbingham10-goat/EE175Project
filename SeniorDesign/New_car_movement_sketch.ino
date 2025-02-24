// Right Motor connections
int enA = 11;
int in1 = 10;
int in2 = 9;
// Left Motor connections
int enB = 6;
int in3 = 4;
int in4 = 5;

const float Pi = 3.14159;
unsigned long lastDebounceTimeA = 0;
unsigned long lastDebounceTimeB = 0;
const unsigned long debounceDelay = 1000;
int encoder1 = 2; //Left motor encoder
int encoder2 = 3; //Right motor encoder

// Variables to store encoder counts
volatile long leftPulses = 0;
volatile long rightPulses = 0;

String receiveData;
const float wheelCircumference = 21.99; //centimeters
const int pulsesPerRevolution = 20; //# of holes in encoder wheel
const float wheelBase = 18.73;  //distance between wheels in cm
const float wheelDiameter = 7.01;
float angleIn = 0;
int camera = 0;
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
void setup() {
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), countEncoder2, RISING);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  static String command = ""; // Buffer to store incoming characters
  while (Serial.available()) {  // Check if data is available
    char c = Serial.read();   // Read one character
      if (c == '\n') {          // End of command
        processCommand(command);
        command = "";         // Clear the buffer
      } 
      else {
        command += c;         // Append character to the buffer
      }
  }
  
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
        analogWrite(enA, 80);
        digitalWrite(in3, HIGH); 
        digitalWrite(in4, LOW);
        analogWrite(enB, 80);
    } else {  // Rotate Left
        digitalWrite(in1, LOW); 
        digitalWrite(in2, HIGH);
        analogWrite(enA, 80);
        digitalWrite(in3, LOW); 
        digitalWrite(in4, HIGH);
        analogWrite(enB, 80);
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

 
  

