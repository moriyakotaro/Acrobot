#include "BNO055.h"

Bno055GyroSensor::Bno055GyroSensor(Adafruit_BNO055* bno) : bno(bno) {
	membaInit();
}

void Bno055GyroSensor::membaInit(){
  pos_flag=true;
}
void Bno055GyroSensor::init(){
  if (!bno->begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void Bno055GyroSensor::updateGyroData(){
  bno->getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno->getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno->getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno->getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno->getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  boardTemp = bno->getTemp();
  orientationdifference();
  return;
}

void Bno055GyroSensor::orientationdifference(){
	if(pos_flag != true){
		double diff_x, diff_y, diff_z;
        diff_x = orientationData.orientation.x - preorientationData_x;
        diff_y = orientationData.orientation.y - preorientationData_y;
        diff_z = orientationData.orientation.z - preorientationData_z;
        if (diff_x < -(180)){
          diff_x += 360;
        }else if ((180) < diff_x){
          diff_x -= 360;
        }
        if (diff_y < -(180)){
          diff_y += 360;
        }else if ((180) < diff_y){
          diff_y -= 360;
        }
        if (diff_z < -(180)){
          diff_z += 360;
        }else if ((180) < diff_z){
          diff_z -= 360;
        }
        orientationdifference_x += diff_x;
        orientationdifference_y += diff_y;
        orientationdifference_z += diff_z;
	}else{
		orientationdifference_x = 0;
		orientationdifference_y = 0;
		orientationdifference_z = 0;
		pos_flag = false;
	}
	preorientationData_x = orientationData.orientation.x;
	preorientationData_y = orientationData.orientation.y;
	preorientationData_z = orientationData.orientation.z;
}

void Bno055GyroSensor::dispGyroData(){
  Serial.print(orientationdifference_x);
  Serial.print("\t");
  Serial.print(orientationdifference_y);
  Serial.print("\t");
  Serial.println(orientationdifference_z);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno->getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
}

void Bno055GyroSensor::printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

sensors_vec_t Bno055GyroSensor::getOrientation(){
	return orientationData.orientation;
}
double Bno055GyroSensor::getOrientationX(){
	return orientationData.orientation.x;
}
double Bno055GyroSensor::getOrientationY(){
	return orientationData.orientation.y;
}
double Bno055GyroSensor::getOrientationZ(){
	return orientationData.orientation.z;
}
double Bno055GyroSensor::getOrientationdifferenceX(){
	return orientationdifference_x;
}
double Bno055GyroSensor::getOrientationdifferenceY(){
  return orientationdifference_y;
}
double Bno055GyroSensor::getOrientationdifferenceZ(){
  return orientationdifference_z;
}

sensors_vec_t Bno055GyroSensor::getAngVelocity(){
	return angVelocityData.gyro;
}
double Bno055GyroSensor::getAngVelocityX(){
	return angVelocityData.gyro.x;
}
double Bno055GyroSensor::getAngVelocityY(){
	return angVelocityData.gyro.y;
}
double Bno055GyroSensor::getAngVelocityZ(){
	return angVelocityData.gyro.z;
}

sensors_vec_t Bno055GyroSensor::getLinearAccel(){
	return linearAccelData.acceleration;
}
double Bno055GyroSensor::getLinearAccelX(){
  return linearAccelData.acceleration.x;
}
double Bno055GyroSensor::getLinearAccelY(){
  return linearAccelData.acceleration.y;
}
double Bno055GyroSensor::getLinearAccelZ(){
  return linearAccelData.acceleration.z;
}


sensors_vec_t Bno055GyroSensor::getMagnetometer(){
	return magnetometerData.magnetic;
}
double Bno055GyroSensor::getMagnetometerX(){
  return magnetometerData.magnetic.x;
}
double Bno055GyroSensor::getMagnetometerY(){
  return magnetometerData.magnetic.y;
}
double Bno055GyroSensor::getMagnetometerZ(){
  return magnetometerData.magnetic.z;
}

sensors_vec_t Bno055GyroSensor::getAccelerometer(){
	return accelerometerData.acceleration;
}
double Bno055GyroSensor::getAccelerometerX(){
  return accelerometerData.acceleration.x;
}
double Bno055GyroSensor::getAccelerometerY(){
  return accelerometerData.acceleration.y;
}
double Bno055GyroSensor::getAccelerometerZ(){
  return accelerometerData.acceleration.z;
}

sensors_vec_t Bno055GyroSensor::getGravity(){
	return gravityData.acceleration;
}
double Bno055GyroSensor::getGravityX(){
  return gravityData.acceleration.x;
}
double Bno055GyroSensor::getGravityY(){
  return gravityData.acceleration.y;
}
double Bno055GyroSensor::getGravityZ(){
  return gravityData.acceleration.z;
}

int8_t Bno055GyroSensor::getTemperature(){
	return boardTemp;
}