#ifndef _BNO055_H_
#define _BNO055_H_

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class Bno055GyroSensor{
	public:
		
		Bno055GyroSensor(Adafruit_BNO055* bno); //BNO055センサーのオブジェクトアドレス
		
		void init(); //初期化用関数

		void updateGyroData();

		sensors_vec_t getOrientation();
		double getOrientationX();
		double getOrientationY();
		double getOrientationZ();
		double getOrientationX_error();
		double getOrientationY_error();
		double getOrientationZ_error();
		sensors_vec_t getAngVelocity();
		double getAngVelocityX();
		double getAngVelocityY();
		double getAngVelocityZ();
		sensors_vec_t getLinearAccel();
		double getLinearAccelX();
		double getLinearAccelY();
		double getLinearAccelZ();
		sensors_vec_t getMagnetometer();
		double getMagnetometerX();
		double getMagnetometerY();
		double getMagnetometerZ();
		sensors_vec_t getAccelerometer();
		double getAccelerometerX();
		double getAccelerometerY();
		double getAccelerometerZ();
		sensors_vec_t getGravity();
		double getGravityX();
		double getGravityY();
		double getGravityZ();
		int8_t getTemperature();

/////////////////////display data/////////////////////////////////////////////////////////
		void dispGyroData();	
//////////////////////////////////////////////////////////////////////////////////////////

	private:
		Adafruit_BNO055* bno;		// BNO055センサーのオブジェクトアドレス

		sensors_event_t orientationData;	//センサーから取得したオイラー角データをまとめている構造体
		sensors_event_t angVelocityData;	//センサーから取得した角速度データをまとめている構造体
		sensors_event_t linearAccelData;	//センサーから取得した線形加速度データをまとめている構造体
		sensors_event_t magnetometerData;	//センサーから取得した地磁気データをまとめている構造体
		sensors_event_t accelerometerData;	//センサーから取得した加速度データをまとめている構造体
		sensors_event_t gravityData;		//センサーから取得した重力加速度データをまとめている構造体

		int8_t boardTemp;			//センサーから取得した温度データを入れる変数

		double offset_orientation_x;	//オイラー角のX軸のオフセット値
		double offset_orientation_y;	//オイラー角のY軸のオフセット値
		double offset_orientation_z;	//オイラー角のZ軸のオフセット値

		void membaInit();			//変数初期化関数

		void printEvent(sensors_event_t* event);
};

#endif