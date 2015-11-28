#include "gyroscope_ITG3200.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

// Registers
#define REG_DLPF_FS					0x16
#define REG_CONFIG					0x17
#define REG_STATUS					0x1A
#define REG_DATA_X_MSB				0x1D
#define REG_DATA_X_LSB				0x1E
#define REG_DATA_Y_MSB				0x1F
#define REG_DATA_Y_LSB				0x20
#define REG_DATA_Z_MSB				0x21
#define REG_DATA_Z_LSB				0x22
#define REG_POWER_MANAGE			0x3E


// Debug verbose output
#define DEBUG_MODE				1


// Constructor
Gyroscope_ITG3200::Gyroscope_ITG3200()
{
	// Initialization
	m_WiringPi_I2C = -1;


	// Offsets
	m_Offset_X = 0;
	m_Offset_Y = 0;
	m_Offset_Z = 0;
}

// Destructor
Gyroscope_ITG3200::~Gyroscope_ITG3200()
{

}


// Connect to the device on I2C and ensure it exists
int Gyroscope_ITG3200::ConnectToI2C(int address)
{
	// Setup the WiringPi I2C library and connect to the motor controller
	m_WiringPi_I2C = wiringPiI2CSetup(address);
	if (m_WiringPi_I2C == -1)
	{
		if (DEBUG_MODE)
			printf("Gyroscope_ITG3200 - wiringPi setup failed\n");

		return -1;
	}

	if (DEBUG_MODE)
		printf("Gyroscope_ITG3200 - wiringPi connected to sensor\n");

	// Set full scale range, highest sampling
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_DLPF_FS, 0x18);


	// Wait to turn on
	delay(50);

	return 1;
}

void Gyroscope_ITG3200::ZeroCalibrate(int samples, int delayMS)
{
	m_Offset_X = 0;
	m_Offset_Y = 0;
	m_Offset_Z = 0;

	for (int i = 0; i < samples; i ++)
	{
		delay(delayMS);
		Gyroscope_Data gyroData = ReadData();

		m_Offset_X += (float)gyroData.gyro_X;
		m_Offset_Y += (float)gyroData.gyro_Y;
		m_Offset_Z += (float)gyroData.gyro_Z;
	}

	m_Offset_X /= (float)-samples;
	m_Offset_Y /= (float)-samples;
	m_Offset_Z /= (float)-samples;

	if (DEBUG_MODE)
		printf("Gyro Offset X: %.1f   Y: %.1f   Z: %.1f\n", m_Offset_X, m_Offset_Y, m_Offset_Z);
}

// Read in the values for the gyro data and return
Gyroscope_Data Gyroscope_ITG3200::Update()
{
	Gyroscope_Data gyroData = ReadData();

	gyroData.gyro_X = (gyroData.gyro_X + (short)m_Offset_X);// / 14.375;
	gyroData.gyro_Y = (gyroData.gyro_Y + (short)m_Offset_Y);// / 14.375;
	gyroData.gyro_Z = (gyroData.gyro_Z + (short)m_Offset_Z);// / 14.375;

	return gyroData;
}

Gyroscope_Data Gyroscope_ITG3200::ReadData()
{
	Gyroscope_Data gyroData = Gyroscope_Data();

	// Read the data from the registers
	unsigned char xhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_X_MSB);
	unsigned char xlo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_X_LSB);
	unsigned char yhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Y_MSB);
	unsigned char ylo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Y_LSB);
	unsigned char zhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Z_MSB);
	unsigned char zlo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Z_LSB);

	// Combine the data to form the meaurement
	gyroData.gyro_X = (short)(xlo | ((short)xhi << 8));
	gyroData.gyro_Y = (short)(ylo | ((short)yhi << 8));
	gyroData.gyro_Z = (short)(zlo | ((short)zhi << 8));

	return gyroData;
}
