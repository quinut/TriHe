#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float temperature;

// MPU6050
MPU6050 mpu; // MPU6050 - 자이로센서

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];


void setup() {
    Wire.begin();
    Serial.begin(115200);

    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}



void loop() {
  if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // 데이터 패킷 읽기
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // ypr[0]: Yaw, ypr[1]: Pitch, ypr[2]: Roll (라디안)
        Serial.print("Yaw: ");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print(" Pitch: ");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print(" Roll: ");
        Serial.println(ypr[2] * 180/M_PI);
    }
    
}


