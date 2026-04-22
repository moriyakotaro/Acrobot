#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include "CAN.h"
#include "RobomasMotor.h"
#include <MsTimer2.h>
#include <Wire.h>
#include "BNO055.h"

const double motor_control_cycle = 10.0;/*ms*/

PIDGain RpmM3508 = {5., 3., 0.};
PIDGain PosM3508 = {3., 1., 0.};
PIDGain RpmM2006 = {2., 1., 0.};
PIDGain PosM2006 = {3., 1., 0.};
PIDGain RpmGM6020 = {5., 0., 0.};
PIDGain PosGM6020 = {40., 3., 0.};
// LQRGain LQRM2006 = {-7.871, -6.221, -1.157, -1.195};
LQRGain LQRM2006 = {0., 0., 0., 0.};
SenserData Gyro = {0., 0.};

CanControl DriveCan1(1);  //CanContorlクラスの定義　引数に使用するCANbusの番号を入力する
RobomasMotor motor1(&DriveCan1, motor_control_cycle); //CanContorlクラスのアドレス，制御周期(ms)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Bno055GyroSensor gyro1(&bno);

Metro DispTiming(50);

void compute(){
  gyro1.updateGyroData();   //ジャイロデータを取得する
  Gyro.senser1 = gyro1.getOrientationX_error();
  Gyro.senser2 =gyro1.getAngVelocityX();
  for(int i=5; i<=8; i++){
    motor1.setSenserDataM2006(i, &Gyro);
  }
  motor1.Control();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!
  gyro1.init();   /* Initialise the sensor */
  motor1.init();    //何も考えずとりあえず入れてください
  delay(1000);
  MsTimer2::set(motor_control_cycle, compute);  //タイマー割込みの設定　引数（RobomasMotorのクラスの制御周期（㎳）と同じもの　　,　　タイマー割込みさせたい関数のアドレス(このプログラムではcompute()のこと) ）
  MsTimer2::start();  //タイマー割込みを開始する
  for(int i=5; i<=8; i++){
    motor1.setLQRgain(M2006, i, &LQRM2006);
  }
}

void loop() {
  if(DispTiming.check()){
    gyro1.dispGyroData();
  }

}