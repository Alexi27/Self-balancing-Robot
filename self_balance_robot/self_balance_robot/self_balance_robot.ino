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

int counter, state;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

unsigned long debutTimeread = 0;
unsigned long debutTimewrite = 0;
unsigned long timeread = 10000;
unsigned long timewrite = 10;


void setup() {
	
	Serial.begin(115200);
	Wire.begin();
	//To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of codeevery 20us
	//This routine is TIMER2_COMPA_vect
	TCCR2A = 0;                                                              
	TCCR2B = 0;                                                               
	TIMSK2 |= (1 << OCIE2A);                                                 
	TCCR2B |= (1 << CS21);                                                  
	OCR2A = 39;                                                               
	TCCR2A |= (1 << WGM21);                                                   

	/* Set kalman and gyro starting angle */
	while (i2cRead(0x3B, i2cData, 6));
	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	double roll = atan2(accY, accZ) * RAD_TO_DEG;
	kalmanX.setAngle(roll); // Set starting angle
	gyroXangle = roll;
	timer = micros();
	
	Serial.print("START");
}

void loop() {

	
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

	
	
	//Serial.print(roll); Serial.print("\t");
	//Serial.print(gyroXangle); Serial.print("\t");
	//Serial.print(kalAngleX); Serial.print("\t");
	//Serial.print("\r\n");
}

ISR(TIMER2_COMPA_vect){
  PORTB &= 0b00100001;
  if (counter < 10) counter++;
  else{
    counter =0;
    if (state==1){
      PORTD |= 0b01000000;
      PORTB |= 0b00010000;  }
               //Set output 2 high to create a pulse for the stepper controller
    else{ 
      PORTD &= 0b10111111;
      PORTB &= 0b11101111; }
    state =!state;}        //Set output 4 low because the pulse only has to last for 20us
}

/*
PORTB &= 0b11110001;
PORTD |= 0b01000000;
PORTB |= 0b00010000;
delayMicroseconds(500);
PORTD &= 0b10111111; ;
PORTB &= 0b11101111;
delayMicroseconds(500);
*/



