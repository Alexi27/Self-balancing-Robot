/*
 Name:		self_balance_robot.ino
 Created:	24/05/2018 16:47:02
 Author:	alexi
*/

// the setup function runs once when you press reset 
#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>
#include <math.h> 

MPU6050 mpu;

int16_t accY, accZ;
float accAngle;

void setup() {
	Serial.print("init ");
	mpu.initialize();
	Serial.begin(9600);	
	Serial.print("fin init ");
}

// the loop function runs over and over again until power down or reset
void loop() {
	
	accZ = mpu.getAccelerationZ();
	accY = mpu.getAccelerationY();

	accAngle = atan2(accY, accZ)*RAD_TO_DEG;

	if (isnan(accAngle));
	else
		Serial.println(accAngle);
}
