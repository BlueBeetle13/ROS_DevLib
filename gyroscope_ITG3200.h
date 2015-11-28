// Class to control the ITG-3200 Gyroscope sensor
#ifndef GYROSCOPE_ITG3200_H
#define GYROSCOPE_ITG3200_H


// Wiring Pi libraries for accessing I2C
#include <wiringPi.h>
#include <wiringPiI2C.h>


struct Gyroscope_Data
{
	short gyro_X;
	short gyro_Y;
	short gyro_Z;
};

class Gyroscope_ITG3200
{
private:
	int				m_WiringPi_I2C;

	float			m_Offset_X, m_Offset_Y, m_Offset_Z;

	Gyroscope_Data ReadData();

public:
	Gyroscope_ITG3200();
	~Gyroscope_ITG3200();

	// Connect to I2C given the address
	int ConnectToI2C(int address);

	// Zero the device
	void ZeroCalibrate(int samples, int delayMS);

	// Return the gyro data
	Gyroscope_Data Update();
};

#endif
