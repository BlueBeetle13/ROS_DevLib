#include "temperaturePressure_BMP085.h"
#include <stdio.h>
#include <time.h>
#include <math.h>


// Control registers
#define REG_CONTROL           	0xF4
#define REG_CONTROL_OUTPUT_MSB  0xF6
#define REG_CONTROL_OUTPUT_LSB  0xF7
#define REG_CONTROL_OUTPUT_XLSB 0xF8

#define READ_TEMPERATURE        0x2E
#define READ_PRESSURE           0x34

// Debug verbose output
#define DEBUG_MODE				0


// Constructor
TemperaturePressure_BMP085::TemperaturePressure_BMP085()
{
	// Initialization
	m_WiringPi_I2C = -1;

	m_OperatingMode = MODE_STANDARD;
	m_Altitude = 0;
	m_CurrentTemperature = 0;
	m_CurrentPressure = 0;
}

// Destructor
TemperaturePressure_BMP085::~TemperaturePressure_BMP085()
{

}

// Connect to the device on I2C and ensure it exists
int TemperaturePressure_BMP085::ConnectToI2C(int address)
{
	// Setup the WiringPi I2C library and connect to the motor controller
	m_WiringPi_I2C = wiringPiI2CSetup(address);
	if (m_WiringPi_I2C == -1)
	{
		if (DEBUG_MODE)
			printf("TemperaturePressure_BMP085 - wiringPi setup failed\n");

		return -1;
	}

	if (DEBUG_MODE)
		printf("TemperaturePressure_BMP085 - wiringPi connected to sensor\n");

	// Read in the calibration data
	ReadCalibrationData();

	return 1;
}

// Set the mode and altitude
void TemperaturePressure_BMP085::SetModeAndAltitude(char mode, int altitude)
{
	if (m_OperatingMode >= 0 && m_OperatingMode < 4)
		m_OperatingMode = mode;

	m_Altitude = altitude;
}

void TemperaturePressure_BMP085::Update()
{
	// *** Raw Temperature

	// Tell the device we want temperature
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_CONTROL, READ_TEMPERATURE);

	// Wait for processing
	delay(5);

	// Read the raw temperature
	long rawTemperature = (wiringPiI2CReadReg8(m_WiringPi_I2C, REG_CONTROL_OUTPUT_MSB) << 8) |
						  wiringPiI2CReadReg8(m_WiringPi_I2C, REG_CONTROL_OUTPUT_LSB);


	// *** Raw Pressure

	// Tell the device we want pressure
	wiringPiI2CWriteReg8(m_WiringPi_I2C, REG_CONTROL, READ_PRESSURE + (m_OperatingMode << 6));

	// Wait for processing
	if (m_OperatingMode == MODE_ULTRA_LOW_POWER)
		delay(5);
	else if (m_OperatingMode == MODE_STANDARD)
		delay(8);
	else if (m_OperatingMode == MODE_HIGHRES)
		delay(14);
	else if (m_OperatingMode == MODE_ULTRA_HIGHRES)
		delay(26);

	// Read in the raw pressure
	long rawPressure = (wiringPiI2CReadReg8(m_WiringPi_I2C, REG_CONTROL_OUTPUT_MSB) << 16) |
					   (wiringPiI2CReadReg8(m_WiringPi_I2C, REG_CONTROL_OUTPUT_LSB) << 8) |
					   wiringPiI2CReadReg8(m_WiringPi_I2C, REG_CONTROL_OUTPUT_XLSB) |
					   (8 - m_OperatingMode);


	// *** Calculate the true temperature
	long x1 = ((long)rawTemperature - m_AC6) * m_AC5 >> 15;
	long x2 = ((long)m_MC << 11) / (x1 + m_MD);
	long b5 = x1 + x2;

	long trueTemperature = (b5 + 8) >> 4;
	m_CurrentTemperature = ((float)trueTemperature) / 10.0;


	// *** Calculate the true pressure
	long b6 = b5 - 4000;
	x1 = (m_B2 * (b6 * b6 >> 12)) >> 11;
	x2 = m_AC2 * b6 >> 11;
	long x3 = x1 + x2;
	long b3 = ((m_AC1 * 4 + x3) << m_OperatingMode + 2) / 4;
	x1 = m_AC3 * b6 >> 13;
	x2 = (m_B1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	unsigned long b4 = (m_AC4 * (unsigned long)(x3 + 32768)) >> 15;
	unsigned long b7 = ((unsigned long)rawPressure - b3) * (50000 >> m_OperatingMode);
	long p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	long tp = p + ((x1 + x2 + 3791) >> 4);

	m_CurrentPressure = tp / pow((1 - (float)m_Altitude / 4433000), 5.255);


	if (DEBUG_MODE)
	{
		printf("RT: %ld   RP: %ld   TP: %ld   T: %.1f   P: %ld\n", rawTemperature, rawPressure, tp, m_CurrentTemperature, m_CurrentPressure);
	}
}

void TemperaturePressure_BMP085::ReadCalibrationData()
{
	if (m_WiringPi_I2C != -1)
	{
		m_AC1 = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xAA) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xAB));
		m_AC2 = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xAC) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xAD));
		m_AC3 = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xAE) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xAF));

		m_AC4 = (unsigned short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB0) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB1));
		m_AC5 = (unsigned short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB2) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB3));
		m_AC6 = (unsigned short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB4) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB5));

		m_B1 = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB6) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB7));
		m_B2 = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB8) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xB9));

		m_MB = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xBA) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xBB));
		m_MC = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xBC) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xBD));
		m_MD = (short)((wiringPiI2CReadReg8(m_WiringPi_I2C, 0xBE) << 8) | wiringPiI2CReadReg8(m_WiringPi_I2C, 0xBF));

		if (DEBUG_MODE)
		{
			printf("AC1: %d\n", m_AC1);
			printf("AC2: %d\n", m_AC2);
			printf("AC3: %d\n", m_AC3);

			printf("AC4: %d\n", m_AC4);
			printf("AC5: %d\n", m_AC5);
			printf("AC6: %d\n", m_AC6);

			printf(" B1: %d\n", m_B1);
			printf(" B2: %d\n", m_B2);

			printf(" MB: %d\n", m_MB);
			printf(" MC: %d\n", m_MC);
			printf(" MD: %d\n", m_MD);
		}
	}
}
