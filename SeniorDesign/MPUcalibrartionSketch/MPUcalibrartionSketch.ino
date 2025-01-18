#include <Wire.h>
const int MPU = 0x68; // MPU-6050 I2C address

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, Tmp;
long AcX_sum = 0, AcY_sum = 0, AcZ_sum = 0;
long GyX_sum = 0, GyY_sum = 0, GyZ_sum = 0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);

  for (int i = 0; i < 100; i++){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    AcX_sum += AcX;
    AcY_sum += AcY;
    AcZ_sum += AcZ;

    GyX_sum += GyX;
    GyY_sum += GyY;
    GyZ_sum += GyZ;

    Serial.println(i);
    delay(100);
  }

  Serial.print("AcX: ");
  Serial.println(AcX_sum/99);
  Serial.print("Acy: ");
  Serial.println(AcY_sum/99);
  Serial.print("AcZ: ");
  Serial.println(AcZ_sum/99);

  Serial.print("GyX: ");
  Serial.println(GyX_sum/99);
  Serial.print("GyY: ");
  Serial.println(GyY_sum/99);
  Serial.print("Gyz: ");
  Serial.print(GyZ_sum/99);

}

void loop() {

}
