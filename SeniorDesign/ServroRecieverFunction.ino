#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// Structure decleration for the coordinates being sent board to board
typedef struct struct_message{
  float x;
  float y;
} struct_message;

// Creates variable to be used for incoming coords
struct_message incomingCoords;

// Create servo objects
Servo xServo;
Servo yServo;

// PWM pins on the ESP32 for servo control
const int xServoPin = 12;
const int yServoPin = 13;

// Height of laser off of ground for trig calculations
double height = 0.16;

// Initialize values for x and y distances 
double xDistance = 0;
double yDistance = 0;

// Servo positions Initialized 
int xPos = 90;
int yPos = 90;

// Callback functioni for the ESP to ESP board connection
void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incomingCoords, incomingData, sizeof(incomingCoords));
  Serial.print("Received X: "); Serial.print(incomingCoords.x);
  Serial.print(" Y: "); Serial.println(incomingCoords.y);

  //update distance variable
  xDistance = incomingCoords.x;
  yDistance = incomingCoords.y;
}

// Define all functions
int calculateXPos(double xDistance);
int calculateYPos(double yDistance);
double calculateAngle(double axisDistance);
double radiansToDegrees(double radians);
double degreesToRadians(double degrees);


void setup() {
  Serial.begin(115200);

  // Mode for board to board wireless communication
  WiFi.mode(WIFI_MODE_STA);

  // Check if initialization was success
  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW Initialization Failed");
    return;
  }
  esp_now_register_recv_cb(onReceive);


  //attach the servos
  xServo.setPeriodHertz(50);
  yServo.setPeriodHertz(50);
  xServo.attach(xServoPin, 500, 2400);
  yServo.attach(yServoPin, 500, 2400);
}

void loop() {
  // Checks if a negative y value was given, want to make sure not out of range
  if(yDistance < 0) {
    Serial.println("ERROR: Negative yDistances are not allowed.");
    yDistance = 0;
  }

  // Calculate the Servo Positions
  xPos = calculateXPos(xDistance);
  yPos = calculateYPos(yDistance);

  // Move the servos
  xServo.write(xPos);
  yServo.write(yPos);

  delay(15);
}

int calculateXPos(double xDistance) {
    if (xDistance < 0) {
        xPos = 90 - calculateAngle(-xDistance);
    } else if (xDistance > 0) {
        xPos = 90 + calculateAngle(xDistance);
    } else {
        xPos = 90;
    }
    return constrain(xPos, 0, 180);
}

int calculateYPos(double yDistance) {
    if (yDistance == 0) {
        yPos = 90;
    } else {
        yPos = 90 - calculateAngle(yDistance);
    }
    return constrain(yPos, 0, 90);
}

double calculateAngle(double axisDistance) {
    return radiansToDegrees(atan2(axisDistance, height));
}

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}