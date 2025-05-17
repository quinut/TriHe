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
int delayTime = 1000;


float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float temperature;


long US_duration, US_distance;


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

MPU6050 mpu; // MPU6050 - 자이로센서

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

float yaw_rad, pitch_rad, roll_rad;

float vir_front = 0; // 가상 전방

Quaternion q;
VectorFloat gravity;
float ypr[3];

float targetHeight = 0; // 목표 고도

PIDState heightPID = {0, 0}; // 고도 제어용
PIDState yawPID = {0, 0}; // Yaw 제어용
PIDState distancePID = {0, 0}; // 거리 제어용


void setup() {

  default_THROTTLE = 0;
  BLDC1.write(1000);
  BLDC2.write(1000);
  BLDC3.write(1000);

  Serial.begin(115200);
  Wire.begin();
  
  dmpInit();
  

  // NRF24L01
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("라디오 통신 시작");


  // MPU6050


  // SRO4M-2
  pinMode(US_ECHO, OUTPUT);
  pinMode(US_TRIG, INPUT);

  // BLDC
  BLDC1.attach(BLDC_1);
  BLDC2.attach(BLDC_2);
  BLDC3.attach(BLDC_3);

  delay(initialDelay);
}

void dmpInit() {
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

  // NRF24L01
  float height_ctrl = getRadioData(ctrl_height);
  float yaw_ctrl = getRadioData(ctrl_yaw);
  float angle_ctrl = getRadioData(ctrl_angle);
  float distance_ctrl = getRadioData(ctrl_distance);
 
  float yaw_last = yaw_rad; // 이전 yaw값

  // MPU6050
  getGyro();

  float yaw_current = yaw_rad;


  // SRO4M-2
  getDistance();

  // 가상 yaw (LStick X)
  float yaw_ctrl_weight = 0.5; // yaw 컨트롤러 가중치
  vir_front = vir_front + angle_ctrl * yaw_ctrl_weight;


  if (height_ctrl > 30) {
    targetHeight += 1; // 상승
  } else if (height_ctrl < -30) {
    targetHeight -= 1; // 하강
  }

  float heightOutput = computePID(
    targetHeight, US_distance,
    2.0, 0.5, 1.0, // Kp, Ki, Kd
    heightPID,
    delayTime / 1000.0, // millis() 로 바꿔서 따로 루프 돌려야 할 듯
    0, 255
  );

  default_THROTTLE = heightOutput;


  float yaw_differ = yaw_current - yaw_last;
  if (yaw_differ < 0) {
    yaw_differ = yaw_differ + 360;
  } else if (yaw_differ > 360) {
    yaw_differ = yaw_differ - 360;
  }
  vir_front = vir_front + yaw_differ; // 회전각도 업데이트

  BLDC_1_THROTTLE = default_THROTTLE - distance_ctrl * cos( angle_ctrl - radians( 0 + vir_front) );
  BLDC_2_THROTTLE = default_THROTTLE - distance_ctrl * cos( angle_ctrl - radians( 120 + vir_front) );
  BLDC_3_THROTTLE = default_THROTTLE - distance_ctrl * cos( angle_ctrl - radians( 240 + vir_front) ); // 각도별 가중치


  // 모터 출력
  BLDC1.write(BLDC_1_THROTTLE);
  BLDC2.write(BLDC_2_THROTTLE);
  BLDC3.write(BLDC_3_THROTTLE);


  delay(delayTime);
}



// NRF24L01 - 데이터 수신
enum dataname{
  ctrl_height,
  ctrl_yaw,
  ctrl_angle,
  ctrl_distance
};
float getRadioData(enum dataname name){
  if (radio.available()) {
    float data[4] = {0};
    radio.read(&data, sizeof(data));
    switch (name) {
      case ctrl_height:
        return data[0];
        break;
      case ctrl_yaw:
        return data[1];
        break;
      case ctrl_angle:
        return data[2];
        break;
      case ctrl_distance:
        return data[3];
        break;
    }
    return data[0];
    // Serial.print("Received: ");
    // for (int i = 0; i < 4; i++) {
    //   Serial.print(data[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();
  }
}

// MPU6050 - 자이로 데이터를 변수에 저장
void getGyro(){
  if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // 데이터 패킷 읽기
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // ypr[0]: Yaw, ypr[1]: Pitch, ypr[2]: Roll (라디안)
        yaw_rad = ypr[0];
        pitch_rad = ypr[1];
        roll_rad = ypr[2];
    }
}


// SRO4M-2 - 거리 측정
void getDistance(){
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  US_duration = pulseIn(US_ECHO, HIGH);
  US_distance = US_duration * 17 / delayTime;
}

// PID
struct PIDState {
  float prevError;
  float integral;
};

float computePID(
  float setpoint,         // 목표값
  float input,            // 현재값
  float Kp, float Ki, float Kd, // PID 계수
  PIDState &state,        // 상태 변수 (참조로 전달)
  float dt,               // 시간 간격(초)
  float outputMin = -255, // 출력 최소값 (기본값)
  float outputMax = 255   // 출력 최대값 (기본값)
) {
  float error = setpoint - input;
  state.integral += error * dt;
  float derivative = (error - state.prevError) / dt;
  float output = Kp * error + Ki * state.integral + Kd * derivative;
  state.prevError = error;
  // 출력값 제한
  if (output > outputMax) output = outputMax;
  if (output < outputMin) output = outputMin;
  return output;
}