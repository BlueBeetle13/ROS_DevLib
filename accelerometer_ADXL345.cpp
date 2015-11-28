#include "accelerometer_ADXL345.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

// Registers
#define REG_DATA_OFFSET_X			0x1E
#define REG_DATA_OFFSET_Y			0x1F
#define REG_DATA_OFFSET_Z			0x20
#define REG_POWER_MODE_DATA_RATE	0x2C
#define REG_POWER_SAVING_MODE		0x2D
#define REG_DATA_FORMAT				0x31
#define REG_DATA_X_LSB				0x32
#define REG_DATA_X_MSB				0x33
#define REG_DATA_Y_LSB				0x34
#define REG_DATA_Y_MSB				0x35
#define REG_DATA_Z_LSB				0x36
#define REG_DATA_Z_MSB				0x37
#define REG_FIFO_CONTROL			0x38
#define REG_FIFO_STATUS				0x39

// Noise level to ignore measurements if below
//#define MIN_NOISE_LEVEL				3


// Debug verbose output
#define DEBUG_MODE				1

// Thread for reading in the accel data
/*void *Accelerometer_ADXL345::ReadAccelDataThreadFunction(void *arg)
{
	Accelerometer_ADXL345 *accelClass = (Accelerometer_ADXL345 *)arg;

	short offset_X = 0;
	short offset_Y = 0;
	short offset_Z = 0;
	bool offsetHasBeenSet = false;

	long deltaAccel_X = 0;
	long deltaAccel_Y = 0;
	long deltaAccel_Z = 0;

	while (!accelClass->m_DataThreadKill)
	{
		// Read the data from the registers
		unsigned char xhi = wiringPiI2CReadReg8(accelClass->m_WiringPi_I2C, REG_DATA_X_MSB);
		unsigned char xlo = wiringPiI2CReadReg8(accelClass->m_WiringPi_I2C, REG_DATA_X_LSB);
		unsigned char yhi = wiringPiI2CReadReg8(accelClass->m_WiringPi_I2C, REG_DATA_Y_MSB);
		unsigned char ylo = wiringPiI2CReadReg8(accelClass->m_WiringPi_I2C, REG_DATA_Y_LSB);
		unsigned char zhi = wiringPiI2CReadReg8(accelClass->m_WiringPi_I2C, REG_DATA_Z_MSB);
		unsigned char zlo = wiringPiI2CReadReg8(accelClass->m_WiringPi_I2C, REG_DATA_Z_LSB);

		// Combine the data to form the meaurement
		short accel_X = (short)(xlo | ((short)xhi << 8));
		short accel_Y = (short)(ylo | ((short)yhi << 8));
		short accel_Z = (short)(zlo | ((short)zhi << 8));

		// Make sure the offset has been set
		if (!offsetHasBeenSet)
		{
			offset_X = accel_X;
			offset_Y = accel_Y;
			offset_Z = accel_Z;

			if (DEBUG_MODE)
				;//printf("Offset X:%d Y:%d Z:%d\n", offset_X, offset_Y, offset_Z);

			offsetHasBeenSet = true;
		}

		// Calculate the deltas - removing noise
		if (abs(accel_X - offset_X) > MIN_NOISE_LEVEL)
			deltaAccel_X += (accel_X - offset_X);

		if (abs(accel_Y - offset_Y) > MIN_NOISE_LEVEL)
			deltaAccel_Y += (accel_Y - offset_Y);

		if (abs(accel_Z - offset_Z) > MIN_NOISE_LEVEL)
			deltaAccel_Z += (accel_Z - offset_Z);


		if (DEBUG_MODE)
			;//printf("X:%d Y:%d Z:%d\n", accel_X - offset_X, accel_Y - offset_Y, accel_Z - offset_Z);

		//printf("%d\n", accel_Y - offset_Y);

		// Report the accel changes
		if (accelClass->m_DataThreadReport)
		{
			// Set the changes
			accelClass->m_DeltaAccel_X = deltaAccel_X;
			accelClass->m_DeltaAccel_Y = deltaAccel_Y;
			accelClass->m_DeltaAccel_Z = deltaAccel_Z;

			// Clear the stored change
			deltaAccel_X = 0;
			deltaAccel_Y = 0;
			deltaAccel_Z = 0;

			accelClass->m_DataThreadReport = false;
		}


		// Delay 40ms because the freq is 25Hz
		delay(40);
	}

	pthread_exit(NULL);

	return NULL;
}*/


// Constructor
Accelerometer_ADXL345::Accelerometer_ADXL345()
{
	// Initialization
	m_WiringPi_I2C = -1;

	// Offset
	m_Offset_X = 0;
	m_Offset_Y = 0;
	m_Offset_Z = 0;

	// Thread control
	//m_DataThreadKill = false;
	//m_DataThreadReport = false;
}

// Destructor
Accelerometer_ADXL345::~Accelerometer_ADXL345()
{
	//m_DataThreadKill = true;
}


// Connect to the device on I2C and ensure it exists
int Accelerometer_ADXL345::ConnectToI2C(int address)
{
	// Setup the WiringPi I2C library and connect to the motor controller
	m_WiringPi_I2C = wiringPiI2CSetup(address);
	if (m_WiringPi_I2C == -1)
	{
		if (DEBUG_MODE)
			printf("Accelerometer_ADXL345 - wiringPi setup failed\n");

		return -1;
	}

	if (DEBUG_MODE)
		printf("Accelerometer_ADXL345 - wiringPi connected to sensor\n");

	// Set to standby mode
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_POWER_SAVING_MODE, 0x00);

	// Set the power mode and measurement mode
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_POWER_SAVING_MODE, 0x08);

	// Set the data rate to 100 Hz
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_POWER_MODE_DATA_RATE, 0x0A); // 0x0A = 100Hz, 0x08 = 25Hz

	// Set the data format +- 16g
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_DATA_FORMAT, 0x08); // 0x03 +- 16, 0x08 +- 2, full res

	// Set the FIFO control - no FIFO
	wiringPiI2CWriteReg8(m_WiringPi_I2C,REG_FIFO_CONTROL, 0x00);

	// Start reading in the data
	//pthread_create(&m_ReadAccelDataThread, NULL, &Accelerometer_ADXL345::ReadAccelDataThreadFunction, this);

	return 1;
}

void Accelerometer_ADXL345::ZeroCalibrate(int samples, int delayMS)
{
	m_Offset_X = 0;
	m_Offset_Y = 0;
	m_Offset_Z = 0;

	for (int i = 0; i < samples; i ++)
	{
		delay(delayMS);
		Accelerometer_Data accelData = ReadData();

		m_Offset_X += (float)accelData.accel_X;
		m_Offset_Y += (float)accelData.accel_Y;
		m_Offset_Z += (float)accelData.accel_Z;
	}

	m_Offset_X /= (float)-samples;
	m_Offset_Y /= (float)-samples;
	m_Offset_Z /= (float)-samples;

	if (DEBUG_MODE)
		printf("Accel Offset X: %.1f   Y: %.1f   Z: %.1f\n", m_Offset_X, m_Offset_Y, m_Offset_Z);
}

// Read in the values for the accel data and return
Accelerometer_Data Accelerometer_ADXL345::Update()
{
	Accelerometer_Data accelData = ReadData();

	accelData.accel_X = (accelData.accel_X + (short)m_Offset_X);
	accelData.accel_Y = (accelData.accel_Y + (short)m_Offset_Y);
	accelData.accel_Z = (accelData.accel_Z + (short)m_Offset_Z);

	/*m_DataThreadReport = true;

	// Wait for data to report
	while (m_DataThreadReport)
		delay(5);

	// Data is ready
	accelData.accel_X = m_DeltaAccel_X;
	accelData.accel_Y = m_DeltaAccel_Y;
	accelData.accel_Z = m_DeltaAccel_Z;*/

	return accelData;
}

Accelerometer_Data Accelerometer_ADXL345::ReadData()
{
	Accelerometer_Data accelData = Accelerometer_Data();

	// Read the data from the registers
	unsigned char xhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_X_MSB);
	unsigned char xlo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_X_LSB);
	unsigned char yhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Y_MSB);
	unsigned char ylo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Y_LSB);
	unsigned char zhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Z_MSB);
	unsigned char zlo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Z_LSB);

	// Combine the data to form the meaurement
	accelData.accel_X = (short)(xlo | ((short)xhi << 8));
	accelData.accel_Y = (short)(ylo | ((short)yhi << 8));
	accelData.accel_Z = (short)(zlo | ((short)zhi << 8));

	return accelData;
}
