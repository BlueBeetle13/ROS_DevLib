#include "analogToDigital_MCP3008.h"
#include <stdio.h>

// Debug verbose output
#define DEBUG_MODE				1


// Constructor
AnalogToDigital_MCP3008::AnalogToDigital_MCP3008()
{
	// Initialization
	m_WiringPi_SPI = -1;
	m_Channel_SPI = -1;
}

// Destructor
AnalogToDigital_MCP3008::~AnalogToDigital_MCP3008()
{

}


// Connect to the device on I2C and ensure it exists
int AnalogToDigital_MCP3008::ConnectToSPI(int spiChannel)
{
	// Setup the WiringPi I2C library and connect to the motor controller
	m_WiringPi_SPI = wiringPiSPISetup(spiChannel, 1000000);
	if (m_WiringPi_SPI == -1)
	{
		if (DEBUG_MODE)
			printf("AnalogToDigital_MCP3008 - wiringPi setup failed\n");

		return -1;
	}

	if (DEBUG_MODE)
		printf("AnalogToDigital_MCP3008 - wiringPi connected to device\n");

	m_Channel_SPI = spiChannel;

	return 1;
}

short AnalogToDigital_MCP3008::ReadPin(unsigned char pinNum)
{
	unsigned char spiData[3];
	unsigned char chanBits;

	chanBits = 0b10000000 | (pinNum << 4);

	spiData [0] = 1;
	spiData [1] = chanBits;
	spiData [2] = 0;

	wiringPiSPIDataRW (m_Channel_SPI, spiData, 3) ;

	return ((spiData [1] << 8) | spiData [2]) & 0x3FF;
}
