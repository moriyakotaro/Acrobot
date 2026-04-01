#include <Arduino.h>
#include <Encoder.h>
#include <Metro.h>
#include <MsTimer2.h>
#include "CAN.h"
#include "RobomasMotor.h"

// External encoder pins
constexpr uint8_t ENC_A = 17;
constexpr uint8_t ENC_B = 27;
constexpr uint8_t ENC_2A = 22;
constexpr uint8_t ENC_2B = 5;
constexpr uint8_t ENC_MA = 6;
constexpr uint8_t ENC_MB = 13;

constexpr double MOTOR_CONTROL_CYCLE_MS = 3.0;  // RobomasMotor control interrupt period
constexpr float DT_SEC = 0.008f;                // LQR update period
constexpr float TWO_PI_F = 2.0f * PI;

// M2006 settings (same CAN control style as main.cpp)
constexpr uint8_t M2006_ID = 5;
constexpr int16_t MAX_TARGET_RPM = 3000;

Encoder enc1(ENC_A, ENC_B);
Encoder enc2(ENC_2A, ENC_2B);
Encoder encm(ENC_MA, ENC_MB);

CanControl DriveCan1(1);
RobomasMotor motor1(&DriveCan1, MOTOR_CONTROL_CYCLE_MS);
PIDGain RpmM2006 = {2.0f, 1.0f, 0.0f};

Metro lqrTiming(8);

float rang1 = 0.0f, rangm = 0.0f;
float prang1 = 0.0f, prangm = 0.0f;
float pprang1 = 0.0f, pprangm = 0.0f;
float drang1 = 0.0f, drang2 = 0.0f, drangm = 0.0f;
float gains[4]{};
float iTQ = 0.0f;
float Vtg = 0.0f;
bool conts = true;
int flag = 0;

void compute() {
  motor1.Control();
}

int16_t voltageToTargetRpm(float voltage) {
  // voltage command -> target RPM for M2006 speed control
  float rpm = (voltage / 24.0f) * static_cast<float>(MAX_TARGET_RPM);
  if (rpm > MAX_TARGET_RPM) rpm = MAX_TARGET_RPM;
  if (rpm < -MAX_TARGET_RPM) rpm = -MAX_TARGET_RPM;
  return static_cast<int16_t>(rpm);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  motor1.init();
  delay(100);

  MsTimer2::set(MOTOR_CONTROL_CYCLE_MS, compute);
  MsTimer2::start();

  motor1.setRpmPIDgain(M2006, M2006_ID, &RpmM2006);

  gains[0] = -0.1546f;
  gains[1] = -7.982f;
  gains[2] = -0.4626f;
  gains[3] = -1.183f;

  enc1.write(0);
  enc2.write(0);
  encm.write(0);

  Serial.println("Pendulum LQR -> M2006 RPM control start");
}

void loop() {
  if (!lqrTiming.check()) return;

  if (!conts) {
    motor1.setTargetRpmM2006(M2006_ID, 0);
    return;
  }

  const float angle = -(static_cast<float>(enc1.read()) / 4096.0f) * TWO_PI_F;
  const float angle2 = -(static_cast<float>(enc2.read()) / 4096.0f) * TWO_PI_F;
  const float anglem = -(static_cast<float>(encm.read()) / 4096.0f) * TWO_PI_F / 28.0f;

  pprang1 = prang1;
  pprangm = prangm;
  prang1 = rang1;
  prangm = rangm;

  rang1 = anglem + angle;
  rangm = anglem;

  drang1 = (rang1 - pprang1) / DT_SEC;
  drang2 = (angle2 - prang1) / DT_SEC;
  drangm = (rangm - pprangm) / DT_SEC;

  iTQ = -(gains[0] * rangm + gains[1] * rang1 + gains[2] * drangm + gains[3] * drang1);
  Vtg = 2.8e+1f * 0.0518f * iTQ + 0.1638f * drangm;

  if (flag == 1) {
    Vtg = 0.0f;
    conts = false;
  }

  const int16_t targetRpm = voltageToTargetRpm(Vtg);
  motor1.setTargetRpmM2006(M2006_ID, targetRpm);

  if (fabsf(anglem) > (TWO_PI_F * 0.5f)) {
    flag = 1;
  }

  Serial.printf(
      "Enc1:%9.5f Enc2:%9.5f Mot:%9.5f dth1:%9.5f dthm:%9.5f V:%7.3f RPM:%6d Torq:%8.5f\r",
      angle, angle2, anglem, drang1, drangm, Vtg, targetRpm, iTQ);
}
