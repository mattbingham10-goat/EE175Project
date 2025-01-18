#include <Wire.h>
#include <math.h>

const int MPU = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double ax, ay, az; // Accelerations in m/s²
double vx = 0, vy = 0, vz = 0; // Velocities in m/s
double px = 0, py = 0, pz = 0; // Positions in m  
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal;
double t, tx, tf, pitch, roll;
unsigned long lastTime, currentTime;
double dt;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  lastTime = micros();
}

void loop() {
  // Request data from module
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  // OFFSETS
  AcXcal = -438;
  AcYcal = -160;
  AcZcal = -16360;
  tcal = -0;
  GyXcal = 680;
  GyYcal = -30;
  GyZcal = 10;

  // Read data received from module
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  ax = (AcX / 16384.0) * 9.81;
  ay = (AcY / 16384.0) * 9.81;
  az = (AcZ / 16384.0) * 9.81;

  // Calculate temp (PROBABLY DON'T NEED)
  // tx = Tmp + tcal;
  // t = tx / 340 + 36.53;
  // tf = (t * 9 / 5) + 32;

  // Send accelerometer data to the getAngle function to get the angles
  getAngle(AcX, AcY, AcZ);

  // Compensate for gravity
  ax = ax - sin(pitch * (M_PI / 180)) * 9.81;
  ay = ay - sin(roll * (M_PI / 180)) * 9.81;
  az = az - cos(pitch * (M_PI / 180)) * cos(roll * (M_PI / 180)) * 9.81;

  // Calculate time difference
  currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
  lastTime = currentTime;

  // Integrate acceleration to get velocity
  vx += ax * dt;
  vy += ay * dt;
  vz += az * dt;

  // Integrate velocity to get position
  px += vx * dt;
  py += vy * dt;
  pz += vz * dt;

  // Print positions
  Serial.print("Position: ");
  Serial.print("X = ");
  Serial.print(px);
  Serial.print(" m, Y = ");
  Serial.print(py);
  Serial.print(" m, Z = ");
  Serial.println(pz);


  // Print out data
  // Angle
  // Serial.print("Angle: ");
  // Serial.print("Pitch = ");
  // Serial.print(pitch);
  // Serial.print(" Roll = ");
  // Serial.println(roll);

  // Accelerometer
  // Serial.print("Accelerometer: ");
  // Serial.print("X = ");
  //Serial.println(AcX + AcXcal);
  // Serial.print(" Y = ");
  // Serial.print(AcY + AcYcal);
  // Serial.print(" Z = ");
  // Serial.println(AcZ + AcZcal);

  // Temperature
  // Serial.print("Temperature in celsius = ");
  // Serial.print(t);
  // Serial.print(" fahrenheit = ");
  // Serial.println(tf);

  // Gyroscope measurements
  // Serial.print("Gyroscope: ");
  // Serial.print("X = ");
  // Serial.print(GyX + GyXcal);
  // Serial.print(" Y = ");
  // Serial.print(GyY + GyYcal);
  // Serial.print(" Z = ");
  // Serial.println(GyZ + GyZcal);
  delay(100);
}

void getAngle(int Ax, int Ay, int Az) {
  double x = Ax;
  double y = Ay;
  double z = Az;
  pitch = atan(x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  pitch = pitch * (180.0 / M_PI);
  roll = roll * (180.0 / M_PI);
}
