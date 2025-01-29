#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

const float gravity = 9.81;

Adafruit_MPU6050 mpu;
Madgwick filter;               //creates filter instance

void calibrateAccelerometer();// calibration function

// declare variables
//position and velocities
float velX = 0, velY = 0;     //velocities
float posX = 0, posY = 0;     //positions

//timing variables
unsigned long lastTime = 0;   //time tracking
float dt = 0;                 //time step


//off set values to calibrate device
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;


void setup() {
  Serial.begin(19200);

  // initialize the MPU device
  // while also error checking to catch false starts
  if ((!mpu.begin())) {
    Serial.println("Failed to initialze the MPU");
    while(1);
  }
  Serial.println("MPU successfully initialized");


  //set ranges that the sensor can detect
  //this limits accelerometer range to +- 2g (g meaning gravity)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  //allows sensor to measure movement of up to 250 degrees / s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  //this sets the filter bandwidth, this allows us to control what frequencies pass through unattuned to the data, this filters out noise 
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  //initialize timing variable
  lastTime = millis();

  //takes averages of readings and calibrates accordingly, gives OFFSETS
  calibrateAccelerometer();

}

void loop() {

  // time recording, allows us to integrate using dt
  unsigned int currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; //converting to seconds
  lastTime = currentTime;


  //create sensor struct and fill with get event
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // seperate the variables into their components and apply the offset
  float accelX = a.acceleration.x - accelOffsetX;
  float accelY = a.acceleration.y - accelOffsetY;
  float accelZ = a.acceleration.z - accelOffsetZ;

  //pull out gyro readings and convert to radians to use filter
  float gyroX = g.gyro.x * (3.14159 / 180.0);
  float gyroY = g.gyro.y * (3.14159 / 180.0);
  float gyroZ = g.gyro.z * (3.14159 / 180.0);

  //use MADGWICK FILTER
  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

  //gets rotation information
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  //use rotation information to calculate x and y offsets due to gravity
  float gravityX = gravity * sin(pitch * (3.14159 / 180.0));
  float gravityY = gravity * sin(roll * (3.14159 / 180.0));

  accelX -= gravityX;
  accelY -= gravityY;

  // find the velocity by integrating the acceleration
  velX += accelX * dt;
  velY += accelY * dt;


  //THIS FUNCTION PREVENTS DRIFT WHILE DEVICE IS STATIONARY thresholds can be tweaked if it seems like its not helping
  if (abs(accelX) < 0.1 && abs(accelY) < 0.05) { // Small acceleration threshold
    velX = 0;
    velY = 0;
  }

  // find the position by integrating the velocity
  posX += velX * dt;
  posY += velY * dt;

  //print out values 
  Serial.print("Position X: ");
  Serial.print(posX);
  Serial.print(" m, Position Y: ");
  Serial.print(posY);
  Serial.println(" m");

  //small delay added just for stability
  delay(10);

}

void calibrateAccelerometer() {
  int numSamples = 100;
  float sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("Calibrating Sensor with Average Stationary Values...");

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z - gravity;      //because gravity is constant in Z direction we can take it off here

    delay(10);
  }

  //calculates average offsets
  accelOffsetX = sumX / numSamples;
  accelOffsetY = sumY / numSamples;
  accelOffsetZ = sumZ / numSamples;

  Serial.println("Calibration Complete...");
}
