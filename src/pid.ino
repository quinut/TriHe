// PID 제어 알고리즘 구현 예시


PIDState altitudePID = {0, 0}; // 고도 제어용
PIDState rollPID = {0, 0};     // 롤 제어용

void loop() {
  float dt = 0.05; // 샘플링 주기(초) 예시

  // 예시: 고도 제어
  float altitudeOutput = computePID(
    targetAltitude, currentAltitude,
    2.0, 0.5, 1.0, // Kp, Ki, Kd
    altitudePID,
    dt,
    0, 255
  );

  // 예시: 롤(기울기) 제어
  float rollOutput = computePID(
    targetRoll, currentRoll,
    1.5, 0.3, 0.8, // Kp, Ki, Kd (롤용)
    rollPID,
    dt,
    -100, 100
  );

  // ... 각각의 출력값을 모터 제어나 시스템에 적용
}


// PID 상태를 저장할 구조체 정의
struct PIDState {
  float prevError;
  float integral;
};

// 범용 PID 함수
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
