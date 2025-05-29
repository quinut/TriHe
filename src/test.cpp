#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#include <nRF24L01.h>
#include <RF24.h> //NRF24L01 - 수신기

#include <Servo.h>

// 변수 선언

int initialDelay = 1000;
int delayTime = 100;


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


#define BLDC_1 3 //고정(PWM)
#define BLDC_2 5 //고정(PWM)
#define BLDC_3 6 //고정(PWM)    // ESC


// NRF24L01
RF24 radio(RADIO_CE, RADIO_CSN); 
const uint64_t pipe = 0xE8E8F0F0E1LL; // 수신 주소 설정

// BLDC
Servo BLDC1;
Servo BLDC2;
Servo BLDC3;

float BLDC_1_THROTTLE = 0;
float BLDC_2_THROTTLE = 0;
float BLDC_3_THROTTLE = 0;
float default_THROTTLE = 0; // 기본 스로틀값



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


  delay(initialDelay);
}



void loop() {


  controllerData ctrl = getRadioData();

  // 1. 기본 스로틀 설정
  int rawThrottle = constrain(ctrl.s1_Y, 0, 100);
  default_THROTTLE = map(rawThrottle, 0, 100, 1000, 2000);

  // 2. 방향 벡터 (R스틱 기준)
  float angle_rad = ctrl.s2_rad;           // 방향 (라디안)
  float strength = constrain(ctrl.s2_distance, 0, 100) / 100.0; // [0.0 ~ 1.0]

  // 3. 방향 벡터를 이용해 각 모터 스로틀 조절
  float radius_effect = 100.0f * strength; // 조절 가능한 최대 영향치 (tweak 가능)

  // 모터 위치 (0°, 120°, 240°)
  float angle_1 = 0.0;
  float angle_2 = 2.0 * PI / 3.0;
  float angle_3 = 4.0 * PI / 3.0;

  // 각 모터의 조정값 계산 (기본 스로틀 ± 보정)
  int throttle1 = default_THROTTLE + radius_effect * cos(angle_1 - angle_rad);
  int throttle2 = default_THROTTLE + radius_effect * cos(angle_2 - angle_rad);
  int throttle3 = default_THROTTLE + radius_effect * cos(angle_3 - angle_rad);

  // 범위 제한
  throttle1 = constrain(throttle1, 1000, 2000);
  throttle2 = constrain(throttle2, 1000, 2000);
  throttle3 = constrain(throttle3, 1000, 2000);

  // 모터 출력
  BLDC1.writeMicroseconds(throttle1);
  BLDC2.writeMicroseconds(throttle2);
  BLDC3.writeMicroseconds(throttle3);

  // 시리얼 출력력
  Serial.print("Throttle1: "); Serial.print(throttle1);
  Serial.print(" | Throttle2: "); Serial.print(throttle2);
  Serial.print(" | Throttle3: "); Serial.println(throttle3);

  delay(delayTime);
}




