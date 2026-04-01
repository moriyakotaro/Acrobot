#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include "CAN.h"
#include "RobomasMotor.h"
#include <MsTimer2.h>

const double motor_control_cycle = 3.0;/*ms*/

bool flag = 1;

int hight = 0;

uint8_t data[8];

int16_t rpm;

PIDGain RpmM3508 = {5., 3., 0.};
PIDGain PosM3508 = {3., 1., 0.};
PIDGain RpmM2006 = {2., 1., 0.};
PIDGain PosM2006 = {3., 1., 0.};
PIDGain RpmGM6020 = {5., 0., 0.};
PIDGain PosGM6020 = {40., 3., 0.};

CanControl DriveCan1(1);  //CanContorlクラスの定義　引数に使用するCANbusの番号を入力する
RobomasMotor motor1(&DriveCan1, motor_control_cycle); //CanContorlクラスのアドレス，制御周期(ms)

Metro DispTiming(50);
Metro flipTiming(5000);

void compute(){
  motor1.Control();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  motor1.init();    //何も考えずとりあえず入れてください
  delay(100);
  MsTimer2::set(motor_control_cycle, compute);  //タイマー割込みの設定　引数（RobomasMotorのクラスの制御周期（㎳）と同じもの　　,　　タイマー割込みさせたい関数のアドレス(このプログラムではcompute()のこと) ）
  MsTimer2::start();  //タイマー割込みを開始する
  for(int i=1; i<=4; i++){
    motor1.setRpmPIDgain(M3508, i, &RpmM3508);
  }
  for(int i=5; i<=8; i++){
    motor1.setRpmPIDgain(M2006, i, &RpmM2006);
  }
}

void loop() {
  if(flipTiming.check()){
    flag = !flag;
  }

  if(flag)  rpm = 1000;
  else      rpm = -1000;


  for(int i=1; i<=4; i++){
    motor1.setTargetRpmM3508(i, rpm);
  }
  for(int i=5; i<=8; i++){
    motor1.setTargetRpmM2006(i, rpm);
  }
}