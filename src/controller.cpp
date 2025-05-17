#include <Arduino.h>
#include <SPI.h>

#include <nRF24L01.h>
#include <RF24.h> //NRF24L01+ - 송송신기

// 핀 선언
#define RADIO_CE 4
#define RADIO_CSN 5
#define RADIO_MOSI 16 //고정
#define RADIO_MISO 14 // 고정
#define RADIO_SCK 15 //고정     // NRF24L01+ - 송신기


#define Stick1_X A0 //아날로그
#define Stick1_Y A1 //아날로그
#define Stick2_X A2 //아날로그
#define Stick2_Y A3 //아날로그   // 조이스틱

#define SWITCH 6               // 스위치

// NRF24L01+
RF24 radio(RADIO_CE, RADIO_CSN); 
const uint64_t pipe = 0xE8E8F0F0E1LL; // 송신 주소 설정

// 변수 선언
int initialDelay = 1000;
int delayTime = 1000;



void setup() {
  Serial.begin(115200);

  // NRF24L01+
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(pipe);
  radio.stopListening();

  // 조이스틱
  
  // 스위치
  pinMode(SWITCH, INPUT_PULLUP);

  delay(initialDelay);
}


void loop() {



  // 조이스틱
  int s1_X = mapJoy(Stick1_X);
  int s1_Y = mapJoy(Stick1_Y);
  int s2_X = mapJoy(Stick2_X);
  int s2_Y = mapJoy(Stick2_Y);

  float s2_rad = atan2(s2_Y, s2_X); // 조이스틱 각도 - 라디안
  float s2_distance = sqrt(s2_X * s2_X + s2_Y * s2_Y); // 조이스틱 거리
  // 거리 매핑
  // s2_distance = s2.distance / ( sqrt(2) * 100 ) * MAPPING_VALUE


   // NRF24L01+ 송신
  float dataSend[5] = {s1_X, s1_Y, s2_rad, s2_distance, checkSwitch()};
  radio.write(dataSend, sizeof(dataSend));


  delay(delayTime);
}




int mapJoy(int pin) {
  int analog = analogRead(pin); 
  int mapped = map(analog, 0, 1023, -100, 100);
  return mapped;
}



//스위치 확인
bool checkSwitch() {
  int switchState = digitalRead(SWITCH);
  if (switchState == LOW) {
    return true;
  } else {
    return false;
  }
}