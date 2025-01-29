#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// declare variables
//position and velocities
float velX = 0, velY = 0;     //velocities
float posX = 0, posY = 0;     //positions

//timing variables
unsigned long lastTime = 0;   //time tracking
float dt = 0;                 //time step




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
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //initialize timing variable
  lastTime = millis();

}

void loop() {
  unsigned int currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; //converting to seconds
  lastTime = currentTime;


  //read the sensor and store in a(acceleration) g(gyro) and temp(temperature)

  //create sensor struct and fill with get event
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // seperate the variables into their x and y components of acceleration
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;


  // we must account for the force of gravity which is a constant that we must factor out of each of the acceleration components
  // THIS COMPENSATION IS NOT ENOUGH NEED KALMAN OR COMBINATION FILTER THING
  float gravity = 9.81;
  accelX -= gravity * sin(g.gyro.x * (3.14159 / 180.0));
  accelY -= gravity * sin(g.gyro.y * (3.14159 / 180.0));

  // find the velocity by integrating the acceleration
  velX += accelX * dt;
  velY += accelY * dt;

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
