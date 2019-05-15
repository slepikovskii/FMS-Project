#include "Uduino.h" 
#include <Adafruit_BNO055.h>

#include <Adafruit_Sensor.h>

#include "I2Cdev.h"

#include "Wire.h"

Uduino uduino("IMU");
Adafruit_BNO055 mpu  = Adafruit_BNO055(55);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars


void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(38400);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

 

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Error
    Serial.println("Error!");
  }
}



void loop() {
  uduino.update();

  if (uduino.isInit()) {
    if (!dmpReady) {
      Serial.println("IMU not connected.");
      delay(10);
      return;
    }

    int  mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check if overflow
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

   
      SendEuler();
      //SendYawPitchRoll();
      //SendRealAccel();
      //SendWorldAccel();
    }
  }


void SendEuler() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  Serial.print(euler[0] * 180 / M_PI); Serial.print("/");
  Serial.print(euler[1] * 180 / M_PI); Serial.print("/");
  Serial.println(euler[2] * 180 / M_PI);
}

void SendYawPitchRoll() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print(ypr[0] * 180 / M_PI); Serial.print("/");
  Serial.print(ypr[1] * 180 / M_PI); Serial.print("/");
  Serial.println(ypr[2] * 180 / M_PI);
}

void SendRealAccel() {
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  Serial.print("a/");
  Serial.print(aaReal.x); Serial.print("/");
  Serial.print(aaReal.y); Serial.print("/");
  Serial.println(aaReal.z);
}

void SendWorldAccel() {
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  Serial.print("a/");
  Serial.print(aaWorld.x); Serial.print("/");
  Serial.print(aaWorld.y); Serial.print("/");
  Serial.println(aaWorld.z);
}
