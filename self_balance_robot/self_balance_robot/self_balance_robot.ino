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
#include "Kalman.h"

MPU6050 mpu;

Kalman kalmanX; // Create the Kalman instances

int i = 0;
				/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

unsigned long debutTimeread = 0;
unsigned long debutTimewrite = 0;
unsigned long timeread = 10000;
unsigned long timewrite = 10;

bool elapsed(unsigned long *previousMillis, unsigned long interval) {
	if (*previousMillis == 0) {
		*previousMillis = micros();
	}
	else {
		if ((unsigned long)(micros() - *previousMillis) >= interval) {
			*previousMillis = 0;
			return true;
		}
	}
	return false;
}
void setup() {
	
	Serial.begin(115200);
	Wire.begin();
	
	/* Set kalman and gyro starting angle */
	while (i2cRead(0x3B, i2cData, 6));
	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	double roll = atan2(accY, accZ) * RAD_TO_DEG;
	kalmanX.setAngle(roll); // Set starting angle
	gyroXangle = roll;
	timer = micros();
	
}

void loop() {
	//if (elapsed(&debutTimeread, timeread)) {
		/* Update all the values */
		while (i2cRead(0x3B, i2cData, 14));
		accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
		accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
		accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
		tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
		gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
		gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
		gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
	
		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		double roll = atan2(accY, accZ) * RAD_TO_DEG;


		double gyroXrate = gyroX / 131.0; // Convert to deg/s
		double gyroYrate = gyroY / 131.0; // Convert to deg/s


		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	


	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter

								  // Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;

	//}
	if (elapsed(&debutTimewrite, timewrite)) {
		digitalWrite(12, !digitalRead(12));
	}
	//Serial.print(roll); Serial.print("\t");
	//Serial.print(gyroXangle); Serial.print("\t");
	//Serial.print(kalAngleX); Serial.print("\t");



	//Serial.print("\r\n");
	//delay(20);
}