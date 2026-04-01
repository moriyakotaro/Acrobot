#include <Arduino.h>
#include <Encoder.h>
#include <SPI.h>

// Teensy 4.1 + MCP4922 settings
constexpr uint8_t DAC_CS_PIN = 10;        // SPI CS pin for MCP4922
constexpr float VREF = 5.34f;             // MCP4922 reference voltage

// Encoder pins
constexpr uint8_t ENC_A = 17;
constexpr uint8_t ENC_B = 27;
constexpr uint8_t ENC_2A = 22;
constexpr uint8_t ENC_2B = 5;
constexpr uint8_t ENC_MA = 6;
constexpr uint8_t ENC_MB = 13;

// Motor direction and debug pins
constexpr uint8_t FR_DCM = 26;
constexpr uint8_t CHECK_PIN = 4;

// Encoder instances (4x decoding in Encoder library)
Encoder enc1(ENC_A, ENC_B);
Encoder enc2(ENC_2A, ENC_2B);
Encoder encm(ENC_MA, ENC_MB);

// LQR state
float rang1 = 0.0f, rang2 = 0.0f, rangm = 0.0f;
float prang1 = 0.0f, prang2 = 0.0f, prangm = 0.0f;
float pprang1 = 0.0f, pprangm = 0.0f;
float drang1 = 0.0f, drang2 = 0.0f, drangm = 0.0f;

float gains[6]{};
float iTQ = 0.0f;
float Vtg = 0.0f;
bool conts = true;
int flag = 0;

constexpr float TWO_PI_F = 2.0f * PI;
constexpr float DT_SEC = 0.008f;  // control period (8 ms)

void writeDac(uint8_t channel, float voltage) {
  if (voltage < 0.0f) voltage = 0.0f;
  if (voltage > VREF) voltage = VREF;

  const uint16_t data = static_cast<uint16_t>((voltage / VREF) * 4095.0f);
  const uint16_t command = (channel == 0) ? 0x3000 : 0xB000;
  const uint16_t value = command | (data & 0x0FFF);

  digitalWrite(DAC_CS_PIN, LOW);
  SPI.transfer16(value);
  digitalWrite(DAC_CS_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // wait for serial monitor on USB, timeout for standalone execution
  }

  pinMode(FR_DCM, OUTPUT);
  pinMode(CHECK_PIN, OUTPUT);
  digitalWrite(FR_DCM, LOW);
  digitalWrite(CHECK_PIN, LOW);

  pinMode(DAC_CS_PIN, OUTPUT);
  digitalWrite(DAC_CS_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // LQR gains
  gains[0] = -0.1546f;
  gains[1] = -7.982f;
  gains[2] = -0.4626f;
  gains[3] = -1.183f;

  enc1.write(0);
  enc2.write(0);
  encm.write(0);

  Serial.println("Teensy 4.1 LQR control start");
}

void loop() {
  if (!conts) {
    writeDac(0, 0.0f);
    return;
  }

  // Encoder library returns quadrature count; keep scale from original source
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
  drangm = (rangm - pprangm) / DT_SEC;

  iTQ = -(gains[0] * rangm + gains[1] * rang1 + gains[2] * drangm + gains[3] * drang1);
  Vtg = 2.8e+1f * 0.0518f * iTQ + 0.1638f * drangm;

  if (flag == 1) {
    Vtg = 0.0f;
  }

  digitalWrite(FR_DCM, (Vtg > 0.0f) ? HIGH : LOW);

  // Keep original forced test voltage behavior
  Vtg = 0.9f;

  if (flag == 1) {
    Vtg = 0.0f;
    conts = false;
  }

  writeDac(0, fabsf(Vtg));

  if (fabsf(anglem) > (TWO_PI_F * 0.5f)) {
    flag = 1;
  }

  Serial.printf(
      "Enc1: %10.5f rad  Enc2: %10.5f rad  Mot: %10.5f rad  dth1: %10.5f rad/s  dth2: %10.5f rad/s  dth0: %10.5f rad/s  Vol: %6.3f V  Torq: %8.5f Nm\r",
      angle, angle2, anglem, drang1, drang2, drangm, Vtg, iTQ);

  delay(8);
}
