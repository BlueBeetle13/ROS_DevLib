// Class to control the MCP3008 analog to digital convertor
#ifndef ANALOG_TO_DIGITAL_MCP3008_H
#define ANALOG_TO_DIGITAL_MCP3008_H


// Wiring Pi libraries for accessing I2C
#include <wiringPi.h>
#include <wiringPiSPI.h>


class AnalogToDigital_MCP3008
{
private:
	int				m_WiringPi_SPI;
	int				m_Channel_SPI;

public:
	AnalogToDigital_MCP3008();
	~AnalogToDigital_MCP3008();

	// Connect to SPI
	int ConnectToSPI(int spiChannel);

	// Read one of the pins
	short ReadPin(unsigned char pinNum);
};

#endif
