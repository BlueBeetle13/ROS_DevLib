#include "magnetometer_HMC5883L.h"
#include <stdio.h>
#include <time.h>
#include <math.h>

// Registers
#define REG_CONFIGURE_A			0x00
#define REG_CONFIGURE_B			0x01
#define REG_MODE				0x02
#define REG_DATA_X_MSB			0x03
#define REG_DATA_X_LSB			0x04
#define REG_DATA_Y_MSB			0x05
#define REG_DATA_Y_LSB			0x06
#define REG_DATA_Z_MSB			0x07
#define REG_DATA_Z_LSB			0x08
#define REG_STATUS				0x09

// Debug verbose output
#define DEBUG_MODE				1


// Constructor
Magnetometer_HMC5883L::Magnetometer_HMC5883L()
{
	// Initialization
	m_WiringPi_I2C = -1;

	// Default to continuous mode
	m_MeasurementMode = MODE_CONTINUOUS;
	m_DeclinationAngle = 0;
}

// Destructor
Magnetometer_HMC5883L::~Magnetometer_HMC5883L()
{

}

// Connect to the device on I2C and ensure it exists
int Magnetometer_HMC5883L::ConnectToI2C(int address)
{
	// Setup the WiringPi I2C library and connect to the motor controller
	m_WiringPi_I2C = wiringPiI2CSetup(address);
	if (m_WiringPi_I2C == -1)
	{
		if (DEBUG_MODE)
			printf("Magnetometer_HMC5883L - wiringPi setup failed\n");

		return -1;
	}

	if (DEBUG_MODE)
		printf("Magnetometer_HMC5883L - wiringPi connected to sensor\n");

	// Set the measurement mode
	SetMeasurementMode(m_MeasurementMode);

	return 1;
}

void Magnetometer_HMC5883L::SetMeasurementMode(unsigned char mode)
{
	if (mode == MODE_CONTINUOUS || mode == MODE_SINGLE || mode == MODE_IDLE)
	{
		m_MeasurementMode = mode;

		// Measurement rate, 15 Hz
		wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_CONFIGURE_A, 0x70);

		// Set the Gain
		wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_CONFIGURE_B, 0x20);

		// Continuous mode (single mode is set for every read)
		if (m_MeasurementMode != MODE_SINGLE)
		{
			wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE, m_MeasurementMode);

			if (m_MeasurementMode == MODE_CONTINUOUS)
				delay(6);
		}
	}
}

void Magnetometer_HMC5883L::SetDeclinationAngle(float angle)
{
	m_DeclinationAngle = angle;
}

Magnetometer_Data Magnetometer_HMC5883L::Update()
{
	// Single measurement mode - we need to set for every read
	if (m_MeasurementMode == MODE_SINGLE)
	{
		wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_MODE, m_MeasurementMode);

		// Wait time
		delay(6);
	}

	Magnetometer_Data magData = Magnetometer_Data();

	// Read the data from the registers
	unsigned char xhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_X_MSB);
	unsigned char xlo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_X_LSB);
	unsigned char yhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Y_MSB);
	unsigned char ylo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Y_LSB);
	unsigned char zhi = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Z_MSB);
	unsigned char zlo = wiringPiI2CReadReg8(m_WiringPi_I2C, REG_DATA_Z_LSB);

	// Combine the data to form the meaurement
	magData.magnet_X = (short)(xlo | ((short)xhi << 8));
	magData.magnet_Y = (short)(ylo | ((short)yhi << 8));
	magData.magnet_Z = (short)(zlo | ((short)zhi << 8));

	// Calculate the heading - using the default scale for the default gain
	//float heading = atan2(((float)magData.y_AxisRaw) * 0.92, ((float)magData.x_AxisRaw) * 0.92); // This is used if the board is rotated with Z pointing up
	float heading = atan2(((float)magData.magnet_Z) * 0.92, ((float)magData.magnet_X) * 0.92); // The board is lying flat

	// Adjust for the declination angle
	heading += m_DeclinationAngle;

	// Correct angle for wrapping
	if(heading < 0)
		heading += 2 * M_PI;

	if(heading > 2 * M_PI)
		heading -= 2 * M_PI;

	// Convert from radians to degrees
	magData.headingDegrees = heading * 180.0 / M_PI;

	return magData;
}
