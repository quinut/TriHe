#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h" // MPU6050 - 자이로센서


#include <nRF24L01.h>
#include <RF24.h> //NRF24L01 - 수신기

#include <Servo.h>

// 변수 선언

int initialDelay = 1000;
int delayTime = 100;


float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float temperature;


long US_duration, US_distance;

// PID
struct PIDState {
  float prevError;
  float integral;
};

struct controllerData {
  int s1_X;
  int s1_Y;
  float s2_rad;
  float s2_distance;
  bool switchState;
};


enum dataname{
  ctrl_height,
  ctrl_yaw,
  ctrl_angle,
  ctrl_distance
};



// 핀 선언
#define RADIO_CE 8
#define RADIO_CSN 9
#define RADIO_MOSI 11 //고정
#define RADIO_MISO 12 // 고정
#define RADIO_SCK 13 //고정     // NRF24L01 - 수신기


#define MPU_SCL A5 //아날로그
#define MPU_SDA A4 //아날로그   // MPU6050 - 자이로센서


#define US_TRIG 0
#define US_ECHO 1              // SRO4M-2 - 거리측정기


#define BLDC_1 3 //고정(PWM)
#define BLDC_2 5 //고정(PWM)
#define BLDC_3 6 //고정(PWM)    // ESC


// NRF24L01
RF24 radio(RADIO_CE, RADIO_CSN); 
const uint64_t pipe = 0xE8E8F0F0E1LL; // 수신 주소 설정


// MPU6050
MPU6050 mpu; // MPU6050 - 자이로센서

// BLDC
Servo BLDC1;
Servo BLDC2;
Servo BLDC3;

float BLDC_1_THROTTLE = 0;
float BLDC_2_THROTTLE = 0;
float BLDC_3_THROTTLE = 0;
float default_THROTTLE = 0; // 기본 스로틀값



float yaw_rad, pitch_rad, roll_rad;




// NRF24L01 - 데이터 수신
controllerData getRadioData() {
  controllerData data; // 기본값(0)으로 초기화
  if (radio.available()) {
    radio.read(&data, sizeof(data));
  }
  return data;
}




void setup() {

  BLDC1.attach(BLDC_1);
  BLDC2.attach(BLDC_2);
  BLDC3.attach(BLDC_3);

  default_THROTTLE = 0;
  BLDC1.write(1000);
  BLDC2.write(1000);
  BLDC3.write(1000);

  Serial.begin(115200);
  Wire.begin();
  
  

  // NRF24L01
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, pipe);
  radio.startListening();
  Serial.println("라디오 통신 시작");


  // SRO4M-2
  pinMode(US_ECHO, OUTPUT);
  pinMode(US_TRIG, INPUT);

  // BLDC


  delay(initialDelay);
}




void loop() {

  // NRF24L01
  float height_ctrl = getRadioData().s1_Y;
  float yaw_ctrl = getRadioData().s1_X;
  float angle_ctrl = getRadioData().s2_rad;
  float distance_ctrl = getRadioData().s2_distance;

  Serial.print("height_ctrl: ");
  Serial.println(height_ctrl);
  Serial.print("yaw_ctrl: ");
  Serial.println(yaw_ctrl);
  Serial.print("angle_ctrl: ");
  Serial.println(angle_ctrl);
  Serial.print("distance_ctrl: ");
  Serial.println(distance_ctrl);
  // Serial.print("US_distance: ");
  // Serial.println(US_distance);
 
 
  
  if (height_ctrl > 30) {
    default_THROTTLE += 100; // 상승
  } else if (height_ctrl < -30) {
    default_THROTTLE -= 100; // 하강
  }
  
  BLDC_1_THROTTLE = default_THROTTLE - distance_ctrl;
  BLDC_2_THROTTLE = default_THROTTLE - distance_ctrl;
  BLDC_3_THROTTLE = default_THROTTLE - distance_ctrl;


  // 모터 출력
  BLDC1.write(BLDC_1_THROTTLE);
  BLDC2.write(BLDC_2_THROTTLE);
  BLDC3.write(BLDC_3_THROTTLE);


  delay(delayTime);
}




