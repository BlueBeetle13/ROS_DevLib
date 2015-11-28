// Class to control the BMP085 Temperature and Pressure sensor
#ifndef TEMPERATURE_PRESSURE_BMP085_H
#define TEMPERATURE_PRESSURE_BMP085_H

// Wiring Pi libraries for accessing I2C
#include <wiringPi.h>
#include <wiringPiI2C.h>


#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25


class TemperaturePressure_BMP085
{
private:
	int		m_WiringPi_I2C;

	int		m_OperatingMode;
	int		m_Altitude;

	// Calibration data
	short	m_AC1, m_AC2, m_AC3;
	unsigned short m_AC4, m_AC5, m_AC6;
	short 	m_B1, m_B2;
	short	m_MB, m_MC, m_MD;

	void ReadCalibrationData();


public:
	TemperaturePressure_BMP085();
	~TemperaturePressure_BMP085();

	float m_CurrentTemperature;
	long m_CurrentPressure;

	// Connect to I2C given the address
	int ConnectToI2C(int address);

	// Set the operating mode and altitude
	void SetModeAndAltitude(char mode, int altitude);

	void Update();
};

#endif
