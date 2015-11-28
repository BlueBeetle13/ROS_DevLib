// Class to control the HMC5883L Magnetometer sensor
#ifndef MAGNETOMETER_HMC5883L_H
#define MAGNETOMETER_HMC5883L_H

// Wiring Pi libraries for accessing I2C
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define MODE_CONTINUOUS			0x00
#define MODE_SINGLE				0x01
#define MODE_IDLE				0x03

struct Magnetometer_Data
{
	short	magnet_X;
	short	magnet_Y;
	short	magnet_Z;

	float	headingDegrees;
};

class Magnetometer_HMC5883L
{
private:
	int				m_WiringPi_I2C;

	unsigned char 	m_MeasurementMode;
	float			m_DeclinationAngle;

public:
	Magnetometer_HMC5883L();
	~Magnetometer_HMC5883L();

	// Connect to I2C given the address
	int ConnectToI2C(int address);

	// Set the mode for taking measurements
	void SetMeasurementMode(unsigned char mode);

	// Declination angle depends on where in the world you are
	void SetDeclinationAngle(float angle);

	// Read in the current data
	Magnetometer_Data Update();
};

#endif
