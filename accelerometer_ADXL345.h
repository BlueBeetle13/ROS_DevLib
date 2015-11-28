// Class to control the ADXL345 Accelerometer sensor
#ifndef ACCELEROMETER_ADXL345_H
#define ACCELEROMETER_ADXL345_H

//#include <pthread.h>

// Wiring Pi libraries for accessing I2C
#include <wiringPi.h>
#include <wiringPiI2C.h>


struct Accelerometer_Data
{
	short accel_X;
	short accel_Y;
	short accel_Z;
};

class Accelerometer_ADXL345
{
private:
	int				m_WiringPi_I2C;

	// Offset
	float			m_Offset_X, m_Offset_Y, m_Offset_Z;

	// Thread
	//bool			m_DataThreadKill;
	//bool			m_DataThreadReport;
	//pthread_t		m_ReadAccelDataThread;

	// Change in accel
	//long			m_DeltaAccel_X, m_DeltaAccel_Y, m_DeltaAccel_Z;

	// Read the data with a thread to fidn the change over a period of time
	//static void *ReadAccelDataThreadFunction(void *arg);

public:
	Accelerometer_ADXL345();
	~Accelerometer_ADXL345();

	// Connect to I2C given the address
	int ConnectToI2C(int address);

	// Data reading thread
	//void ReadAccelDataThreadFunction();

	// Zero calibrate the device
	void ZeroCalibrate(int samples, int delayMS);

	// Return the accel data during the time period between calls to this function
	Accelerometer_Data Update();

	// Read the data from the deice
	Accelerometer_Data ReadData();
};

#endif
