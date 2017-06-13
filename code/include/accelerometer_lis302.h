#pragma once

#include "stm32f4xx.h"
#include "hdr_bitband.h"
#include <math.h>

class AccelerometerLIS302
{

	// Registers of LIS302DL
	static uint16_t const WHO_AM_I = 0x8F00;
	static uint16_t const CTRL_REG1	= 0x2000;
	static uint16_t const CTRL_REG2	= 0x2100;
	static uint16_t const CTRL_REG3 = 0x2200;
	static uint16_t const STATUS_REG = 0xA700;
	static uint16_t const OUTX = 0xA900;
	static uint16_t const OUTY = 0xAB00;
	static uint16_t const OUTZ = 0xAD00;

	static uint8_t const ZYXDA_BIT = 0x08;

	// States of FSM
	enum FsmState
	{
		STATE_IDLE,
		STATE_STATUS,
		STATE_X_AXIS,
		STATE_Y_AXIS,
		STATE_Z_AXIS
	};

	volatile uint16_t u16Data;

	FsmState fsmState;

	static volatile unsigned long & SPI_CS()
	{
		return bitband_t m_BITBAND_PERIPH(&GPIOE->ODR, 3);
	}

	void HardwareInit();

public:

	volatile int8_t rawDataX;
	volatile int8_t rawDataY;
	volatile int8_t rawDataZ;

	float accVal[3];
	float angle[2];

	volatile bool isDataReady;

	// Blocking data exchange on SPI
	uint16_t WriteReadBlock(uint16_t data)
	{
		SPI_CS() = 0;
		__NOP(); __NOP();
		while(!(SPI1->SR & SPI_SR_TXE)) {};
		SPI1->DR = data;
	    while(SPI1->SR & SPI_SR_BSY) {};
	    while(!(SPI1->SR & SPI_SR_RXNE)) {};

	    SPI_CS() = 1;
	    __NOP(); __NOP();
	    data = SPI1->DR;
	    return data;
	}

	void WriteReadStart()
	{
		fsmState = STATE_STATUS;
		SPI_CS() = 0; __NOP(); __NOP();
		// Character to send
		SPI1->DR = STATUS_REG;
	}

	void Init()
	{
		HardwareInit();

		isDataReady = false;
		fsmState = STATE_IDLE;
		u16Data = SPI1->DR;
		// Set accelerometer
		u16Data = WriteReadBlock(WHO_AM_I) & 0xFF;
		u16Data = WriteReadBlock(CTRL_REG1 + 0xC7) & 0xFF; //0x47 100Hz - 0xC7 400Hz
		// Allow for SPI interrupts
		SPI1->CR2 |= SPI_CR2_RXNEIE;
	}

	void Irq()
	{
	    uint16_t data;
		SPI_CS() = 1;
		data = (SPI1->DR) & 0x00FF;
		Fsm(data);
	}

	void Fsm(uint16_t);

	void ScaleData()
	{
		// TO DO: compute acc in m/s^2
		accVal[0] = (float)rawDataX/64.0;
		accVal[1] = (float)rawDataY/64.0;
		accVal[2] = (float)rawDataZ/64.0;
	}

	void CalculateAngles()
	{
		angle[0] = atan2(-accVal[0], accVal[2]);
		angle[1] = atan2(accVal[1], sqrt(accVal[0]*accVal[0] + accVal[2]*accVal[2]));
	}
};

