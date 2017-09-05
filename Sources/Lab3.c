/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <MKL46Z4.h>
#include "slcd.h"


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define PIN(x)		(1<<x)

#define GREEN_LED	(5)
#define RED_LED		(29)

#define SW1		(3)
#define SW3		(12)

#define GREEN_LED_ON()		PTD->PCOR |= PIN(GREEN_LED);
#define GREEN_LED_OFF()		PTD->PSOR |= PIN(GREEN_LED);
#define GREEN_LED_TOGGLE()	PTD->PTOR |= PIN(GREEN_LED);

#define RED_LED_ON()		PTE->PCOR |= PIN(RED_LED);
#define RED_LED_OFF()		PTE->PSOR |= PIN(RED_LED);
#define RED_LED_TOGGLE()	PTE->PTOR |= PIN(RED_LED);

#define I2C_SCL		(24)
#define I2C_SDA		(25)


#define MAG_ID_REG		(0x07)
#define MAG_ADDR		(0x0E)

#define ACC_ID_REG		(0x0D)
#define ACC_ID			(0x1A)
#define ACC_ADDR		(0x1D)

#define MAG_X_MSB_REG	(0x01)
#define MAG_X_LSB_REG	(0x02)
#define MAG_Y_MSB_REG	(0x03)
#define MAG_Y_LSB_REG	(0x04)
#define MAG_Z_MSB_REG	(0x05)
#define MAG_Z_LSB_REG	(0x06)

#define ACC_X_MSB_REG	(0x01)
#define ACC_X_LSB_REG	(0x02)
#define ACC_Y_MSB_REG	(0x03)
#define ACC_Y_LSB_REG	(0x04)
#define ACC_Z_MSB_REG	(0x05)
#define ACC_Z_LSB_REG	(0x06)

#define MAG_CTRL_REG1	(0x10)
#define FR_bit	(0 << 2)
#define AC_bit	(1)

#define PI 3.14159265


signed short MAX_X, MAX_Y, MAX_Z = 0x8000;
signed short MIN_X, MIN_Y, MIN_Z = 32767;
signed short X_OFFSET, Y_OFFSET = 0;

signed short X_ACCEL_OFFSET, Y_ACCEL_OFFSET = 0;

signed short gravity = 0;

short correction = 2;

//MSB and LSB data for 3 DOF
int multiple_byte_array[6] = {0};


typedef enum {
	ACQUIRING,
	RUNNING,
	CALIBRATING,
	ACCEL,
	STOPPED,

} enumCompassOperationState;

enumCompassOperationState CompassState = STOPPED;
int isTimerExpired = 0;


int PT_STATE(x)
{
	unsigned int y = PTC->PDIR & PIN(x);
	return y;
}

void delay(int cycles)
{
	for(int i = 1; i < cycles; i++)
	{

	}
}


unsigned char I2C_SingleByteRead(unsigned char ucAddress, unsigned char ucRegister)
{
	unsigned char ucData = 0;
	//////////////////////////////
//	Clear up previous states
	I2C0->S &= ~I2C_S_TCF_MASK;

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;
	//////////////////////////////
//	Set I2C to transmit mode
	I2C0->C1 |= I2C_C1_TX_MASK;

	//////////////////////////////
//	Start bit generation master mode
//	When changed from 0 to 1, start signal is generated and master mode is selected
//	When changed from 1 to 0, stop signal is generated and mode of oepration changes from master to slave
	I2C0->C1 |= I2C_C1_MST_MASK;

	//////////////////////////////
//	Transmit the address
//	address is 7-bit number and since the MSB is sent first in I2C, must left shift ucAddress to the left
	I2C0->D = (ucAddress << 1);

//	Wait until the transmission is done.
//	TCF = transfer complete flag
//	0 = transfer in progress; 1 = transfer complete
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

//	Wait until Ack/Nack is received
	while((I2C0->S & I2C_S_IICIF_MASK) == 0);

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

	//////////////////////////////
//	Transmit the register
	I2C0->D = ucRegister;

//	Wait until the transmission is done
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

//	Wait until Ack/Nack is received
	while((I2C0->S & I2C_S_IICIF_MASK) == 0);

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

	//////////////////////////////
//	Restart bit generation
	I2C0->C1 |= I2C_C1_RSTA_MASK;

	//////////////////////////////
//	Transmit the address and read
	I2C0->D = ((ucAddress << 1) | 0x01);

//	Wait until the transmission is done
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

//	Wait until Ack/Nack is received, which triggers an interrupt
	while((I2C0->S & I2C_S_IICIF_MASK) == 0);

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

	//////////////////////////////
//	Set I2C to receive mode
	I2C0->C1 &= ~I2C_C1_TX_MASK;

//	No ack is selected
	I2C0->C1 |= I2C_C1_TXAK_MASK;

//	Read dummy byte
	ucData = I2C0->D;

//	Wait until the transmission is done
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

//	Wait until Ack/Nack is received
	while((I2C0->S & I2C_S_IICIF_MASK) == 0);

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

	//////////////////////////////
//	Set I2C to transmit mode
	I2C0->C1 |= I2C_C1_TX_MASK;

//	Read actual data
	ucData = I2C0->D;

	//////////////////////////////
//	Generate Stop bit
	I2C0->C1 &= ~I2C_C1_MST_MASK;

	return ucData;
}

void I2C_MultipleByteRead(unsigned char ucAddress, unsigned char ucRegister, unsigned int numBytes)
{


		multiple_byte_array[6] = 0;

		//////////////////////////////
	//	Clear up previous states
		I2C0->S &= ~I2C_S_TCF_MASK;

	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;
		//////////////////////////////
	//	Set I2C to transmit mode
		I2C0->C1 |= I2C_C1_TX_MASK;

		//////////////////////////////
	//	Start bit generation master mode
	//	When changed from 0 to 1, start signal is generated and master mode is selected
	//	When changed from 1 to 0, stop signal is generated and mode of oepration changes from master to slave
		I2C0->C1 |= I2C_C1_MST_MASK;

		//////////////////////////////
	//	Transmit the address
	//	address is 7-bit number and since the MSB is sent first in I2C, must left shift ucAddress to the left
		I2C0->D = (ucAddress << 1);

	//	Wait until the transmission is done.
	//	TCF = transfer complete flag
	//	0 = transfer in progress; 1 = transfer complete
		while((I2C0->S & I2C_S_TCF_MASK) == 0);

	//	Wait until Ack/Nack is received
		while((I2C0->S & I2C_S_IICIF_MASK) == 0);

	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;

		//////////////////////////////
	//	Transmit the register
		I2C0->D = ucRegister;

	//	Wait until the transmission is done
		while((I2C0->S & I2C_S_TCF_MASK) == 0);

	//	Wait until Ack/Nack is received
		while((I2C0->S & I2C_S_IICIF_MASK) == 0);

	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;

		//////////////////////////////
	//	Restart bit generation
		I2C0->C1 |= I2C_C1_RSTA_MASK;

		//////////////////////////////
	//	Transmit the address and read
		I2C0->D = ((ucAddress << 1) | 0x01);

	//	Wait until the transmission is done
		while((I2C0->S & I2C_S_TCF_MASK) == 0);
	//	Wait until Ack/Nack is received, which triggers an interrupt
		while((I2C0->S & I2C_S_IICIF_MASK) == 0);
	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;

		//////////////////////////////
	//	Set I2C to receive mode
		I2C0->C1 &= ~I2C_C1_TX_MASK;


	for(int i = 0; i < numBytes; i++)
	{

		if(i < numBytes - 1)
		{

			//	Read actual data
				multiple_byte_array[i] = I2C0->D;
			//	send out an ACK
//				I2C0->C1 &= ~I2C_C1_TXAK_MASK;

				//	Wait until the transmission is done
					while((I2C0->S & I2C_S_TCF_MASK) == 0);

				//	Wait until Ack/Nack is received
					while((I2C0->S & I2C_S_IICIF_MASK) == 0);

				//	Clear interrupt flag
					I2C0->S |= I2C_S_IICIF_MASK;



			//	Set I2C to transmit mode
//				I2C0->C1 |= I2C_C1_TX_MASK;

		}

		else if(i == numBytes - 1)
		{
			//	No ack is selected
				I2C0->C1 |= I2C_C1_TXAK_MASK;

				multiple_byte_array[i] = I2C0->D;

			//	Wait until the transmission is done
				while((I2C0->S & I2C_S_TCF_MASK) == 0);

				//	Wait until Ack/Nack is received
				while((I2C0->S & I2C_S_IICIF_MASK) == 0);

				//	Clear interrupt flag
				I2C0->S |= I2C_S_IICIF_MASK;
		}

	}

		//////////////////////////////
	//	Set I2C to transmit mode
		I2C0->C1 |= I2C_C1_TX_MASK;


		//////////////////////////////
	//	Generate Stop bit
		I2C0->C1 &= ~I2C_C1_MST_MASK;



}

void I2C_SingleByteWrite(unsigned char ucAddress, unsigned char ucRegister, unsigned char ucData)
{
//	Clear up previous states
	I2C0->S &= ~I2C_S_TCF_MASK;

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

//	Ack is selected
	I2C0->C1 &= ~I2C_C1_TXAK_MASK;

//	Set I2C to transmit mode
	I2C0->C1 |= I2C_C1_TX_MASK;

//	Start bit generation
	I2C0->C1 |= I2C_C1_MST_MASK;

	//////////////////////////////
//	Transmit the address
	I2C0->D = (ucAddress << 1);

//	Wait until the transmission is done
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

//	Wait until Ack/Nack is received
	while((I2C0->S & I2C_S_IICIF_MASK) == 0);

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

//	Transmit the register
	I2C0->D = ucRegister;

//	Wait until the transmission is done
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

//	Wait until Ack/Nack is received
	while((I2C0->S & I2C_S_IICIF_MASK) == 0);

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

//	Transmit the address to the data register
	I2C0->D = ucData;

//	Wait until the transmission is done
	while((I2C0->S & I2C_S_TCF_MASK) == 0);

//	Wait until Ack/Nack is received
	while((I2C0->S & I2C_S_IICIF_MASK) == 0);

//	Clear interrupt flag
	I2C0->S |= I2C_S_IICIF_MASK;

	//////////////////////////////
//	Set I2C to transmit mode
	I2C0->C1 |= I2C_C1_TX_MASK;

	//////////////////////////////
//	Generate stop bit
	I2C0->C1 &= ~I2C_C1_MST_MASK;

}

void I2C_MultipleByteWrite(unsigned char ucAddress, unsigned char ucRegister, unsigned char ucData, unsigned int numBytes[6])
{


	//	Clear up previous states
		I2C0->S &= ~I2C_S_TCF_MASK;

	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;

	//	Ack is selected
		I2C0->C1 &= ~I2C_C1_TXAK_MASK;

	//	Set I2C to transmit mode
		I2C0->C1 |= I2C_C1_TX_MASK;

	//	Start bit generation
		I2C0->C1 |= I2C_C1_MST_MASK;

		//////////////////////////////
	//	Transmit the address
		I2C0->D = (ucAddress << 1);

	//	Wait until the transmission is done
		while((I2C0->S & I2C_S_TCF_MASK) == 0);

	//	Wait until Ack/Nack is received
		while((I2C0->S & I2C_S_IICIF_MASK) == 0);

	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;

	//	Transmit the register
		I2C0->D = ucRegister;

	//	Wait until the transmission is done
		while((I2C0->S & I2C_S_TCF_MASK) == 0);

	//	Wait until Ack/Nack is received
		while((I2C0->S & I2C_S_IICIF_MASK) == 0);

	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;

	for(int i = 0; i < 6; i++)
	{
	//	Transmit the address to the data register
		I2C0->D = ucData;

	//	Wait until the transmission is done
		while((I2C0->S & I2C_S_TCF_MASK) == 0);

	//	Wait until Ack/Nack is received
		while((I2C0->S & I2C_S_IICIF_MASK) == 0);

	//	Clear interrupt flag
		I2C0->S |= I2C_S_IICIF_MASK;

	}

		//////////////////////////////
	//	Set I2C to transmit mode
		I2C0->C1 |= I2C_C1_TX_MASK;

		//////////////////////////////
	//	Generate stop bit
		I2C0->C1 &= ~I2C_C1_MST_MASK;
}


void SWITCH_Init(void)
{
// --------------------------------------------------------------------
// Place your switch related code and register settings here - Start

	//////////////////// PORT C Stuff


	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

//	set port data direction register to input
	PTC->PDDR &= ~(PIN(SW1));
//  set the SW1,SW3 pin multiplexer to GPIO mode
	PORTC->PCR[SW1] |= PORT_PCR_MUX(1);
//	enable pull-up resistor
	PORTC->PCR[SW1] |= PORT_PCR_PE_MASK;
//	enable pull-up resistor after PE is enabled
	PORTC->PCR[SW1] |= PORT_PCR_PS_MASK;
	//	Interrupt setup
	PORTC->PCR[SW1] |= PORT_PCR_IRQC(0b1010);

//



//	set port data direction register to input

	PTC->PDDR &= ~(PIN(SW3));
	PORTC->PCR[SW3] |= PORT_PCR_MUX(1);
	//	Interrupt setup
	PORTC->PCR[SW3] |= PORT_PCR_IRQC(0b1010);

	//	enable pull-up resistor
	PORTC->PCR[SW3] |= PORT_PCR_PE_MASK;
	//	enable pull-up resistor after PE is enabled
	PORTC->PCR[SW3] |= PORT_PCR_PS_MASK;
// Place your switch related code and register settings here - End
// --------------------------------------------------------------------
}

void TIMER_Init(void)
{
// --------------------------------------------------------------------
// Place your timer related code and register settings here - Start

//		Turns on clock for TPM0
		SIM -> SCGC6 |= SIM_SCGC6_TPM0_MASK;
//		pg 567 Prescale divide 8MHz by 128 = 62500 must be set before counter is enabled
		TPM0 -> SC |= TPM_SC_PS(0b111);
//		Setting the max counter value for TPM module to be 6250,
//		so that the timer overflows every 100 ms instead of 1000 ms so that the main code runs as desired
		TPM0 -> MOD = 62500/10;
//		pg 567 TPM clock mode selection set to TPM counter increments on every TPM counter clock
		TPM0 -> SC |= TPM_SC_CMOD(0b01);
//		Enables TPM overflow interrupts.
		TPM0 -> SC |= TPM_SC_TOIE_MASK;
// 		TPM counter increments even during debug mode
		TPM0 -> CONF |= TPM_CONF_DBGMODE(0b11);

//		pg 206 Selects the clock source for the OSCERCLK counter clock
		SIM -> SOPT2 |= SIM_SOPT2_TPMSRC(0b10);


// Place your timer related code and register settings here - End
// --------------------------------------------------------------------
}

void LED_Init(void)
{
// --------------------------------------------------------------------
// Place your LED related code and register settings here - Start

	//////////////////// PORT D Stuff
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

	//	set the PTD5 pin multiplexer to GPIO mode
	PORTD->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	//	set data direction register for pin 5 on port D to be output
	PTD->PDDR |= PIN(GREEN_LED);
	//	If it's on for some reason, turn it off
	GREEN_LED_OFF();


	//////////////////// PORT E Stuff
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PTE->PDDR |= PIN(RED_LED);

	//	If it's on for some reason, turn it off
	RED_LED_OFF();



// Place your LED related code and register settings here - End
// --------------------------------------------------------------------
}

void MAG_ACQ(void)
{
	int numDOF = 3;

	SLCD_WriteMsg((unsigned char *)"MACQ");

	//	what to write to the control register
	//	Full 16-bit values are read and ACTIVE mode enabled
	int control_register1 = 0;
	control_register1 = FR_bit | AC_bit;

	I2C_SingleByteWrite(MAG_ADDR,MAG_CTRL_REG1,control_register1);

//	Activate magnetic reset
	short control_register2 = (1 << 7) | (1 << 4);
	I2C_SingleByteWrite(MAG_ADDR,0x11,control_register2);


	delay(100);

	while(CompassState == ACQUIRING)
	{
//		When performing a multi-byte or “burst” read, the MAG3110 automatically increments the register address
//		read pointer after a read command is received. multiple bytes of data can be read from
//		sequential registers after each MAG3110 acknowledgment (AK) is received until a no acknowledge (NAK)
//		occurs from the master followed by a stop condition (SP) signaling the end of transmission.
//		Each DOF is encoded in 16-bits, aka 2 bytes per DOF


//		I2C_MultipleByteRead(MAG_ADDR,MAG_X_MSB_REG,numDOF*2);

//		delay(100);

		int id = I2C_SingleByteRead(MAG_ADDR,MAG_ID_REG);
		if(id != 0xC4)
		{
			SLCD_WriteMsg((unsigned char *)"ERROR");
		}


		signed short X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB = 0 ;


		int sanitycheck = I2C_SingleByteRead(MAG_ADDR,0x07);
		delay(100);
//		int sanitycheck2 = I2C_SingleByteRead(MAG_ADDR,MAG_X_LSB_REG);
//		delay(10000);
//		int sanitycheck3 = I2C_SingleByteRead(MAG_ADDR,MAG_X_LSB_REG);
//		delay(10000);
//		int sanitycheck4 = I2C_SingleByteRead(MAG_ADDR,MAG_X_LSB_REG);
//		delay(10000);

		X_MSB = I2C_SingleByteRead(MAG_ADDR,MAG_X_MSB_REG);
		delay(100);
		X_LSB = I2C_SingleByteRead(MAG_ADDR,MAG_X_LSB_REG);
		delay(100);
		Y_MSB = I2C_SingleByteRead(MAG_ADDR,MAG_Y_MSB_REG);
		delay(100);
		Y_LSB = I2C_SingleByteRead(MAG_ADDR,MAG_Y_LSB_REG);
		delay(100);
		Z_MSB = I2C_SingleByteRead(MAG_ADDR,MAG_Z_MSB_REG);
		delay(100);
		Z_LSB = I2C_SingleByteRead(MAG_ADDR,MAG_Z_LSB_REG);
		delay(100);



//		appending LSB to MSB to form 16-bit
//		signed short x_data = X_LSB  | (X_MSB << 8);
//		signed short y_data = Y_LSB | (Y_MSB << 8);
//		signed short z_data = Z_LSB | (Z_MSB << 8);
		signed short x_data = (X_LSB) | (X_MSB << 8);
		signed short y_data = (Y_LSB) | (Y_MSB << 8);
		signed short z_data = (Z_LSB) | (Z_MSB << 8);


		if(x_data > MAX_X)
		{
			MAX_X = x_data;
			GREEN_LED_TOGGLE();
		}

		if(y_data > MAX_Y)
			MAX_Y = y_data;

		if(z_data > MAX_Z)
			MAX_Z = z_data;

		if(x_data < MIN_X)
			MIN_X = x_data;


		if(y_data < MIN_Y)
			MIN_Y = y_data;

		if(z_data < MIN_Z)
			MIN_Z = z_data;

	}

}

void MAG_CAL(void)
{

	SLCD_WriteMsg((unsigned char *)"MCAL");
	while(CompassState == CALIBRATING)
	{
		X_OFFSET = (MIN_X + MAX_X)/2;
		Y_OFFSET = (MIN_Y + MAX_Y)/2;
	}
}

void STOP(void)
{

	while(CompassState == STOPPED)
	{
		SLCD_WriteMsg((unsigned char *)"STOP");
	}

}

void RUN(void)
{
//
	unsigned char LCDmessage[5] = "";

	while(CompassState == RUNNING)
	{
//		if(isTimerExpired == 1)
//		{
//			float x_scale = 1/((float)MAX_X-X_OFFSET);
//			float y_scale = 1/((float)MAX_Y-Y_OFFSET);

//			Accelerometer
			signed short X_MSB_A, X_LSB_A, Y_MSB_A, Y_LSB_A, Z_MSB_A, Z_LSB_A = 0 ;

			X_MSB_A = I2C_SingleByteRead(ACC_ADDR,ACC_X_MSB_REG);
//			delay(10);
			X_LSB_A = I2C_SingleByteRead(ACC_ADDR,ACC_X_LSB_REG);
//			delay(10);
			Y_MSB_A = I2C_SingleByteRead(ACC_ADDR,ACC_Y_MSB_REG);
//			delay(10);
			Y_LSB_A = I2C_SingleByteRead(ACC_ADDR,ACC_Y_LSB_REG);
//			delay(10);
			Z_MSB_A = I2C_SingleByteRead(ACC_ADDR,ACC_Z_MSB_REG);
//			delay(10);
			Z_LSB_A = I2C_SingleByteRead(ACC_ADDR,ACC_Z_LSB_REG);
//			delay(10);

			signed short x_data_A = ((X_MSB_A << 8) | ((X_LSB_A))) >> 2 - X_ACCEL_OFFSET;
			signed short y_data_A = ((Y_MSB_A << 8) | ((Y_LSB_A))) >> 2 - Y_ACCEL_OFFSET;
			signed short z_data_A = ((Z_MSB_A << 8) | ((Z_LSB_A))) >> 2;

			if(z_data_A > 1.1*gravity || z_data_A < 0.9*gravity)
			{
				RED_LED_ON();
				SLCD_WriteMsg((unsigned char *)"ERROR");
			}
			else
			{
				RED_LED_OFF();
			}



//			Magnetometer
			signed short X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB = 0 ;

			X_MSB = I2C_SingleByteRead(MAG_ADDR,MAG_X_MSB_REG);
			delay(100);
			X_LSB = I2C_SingleByteRead(MAG_ADDR,MAG_X_LSB_REG);
			delay(100);
			Y_MSB = I2C_SingleByteRead(MAG_ADDR,MAG_Y_MSB_REG);
			delay(100);
			Y_LSB = I2C_SingleByteRead(MAG_ADDR,MAG_Y_LSB_REG);
			delay(100);
			Z_MSB = I2C_SingleByteRead(MAG_ADDR,MAG_Z_MSB_REG);
			delay(100);
			Z_LSB = I2C_SingleByteRead(MAG_ADDR,MAG_Z_LSB_REG);

//			I2C_MultipleByteRead(MAG_ADDR,MAG_X_MSB_REG,6);

	//		subtracting offsets
			int x_data = ((X_LSB) | (X_MSB << 8) - X_OFFSET);//*x_scale;
			int y_data = (Y_LSB | (Y_MSB << 8) - Y_OFFSET);//*_scale;
			int z_data = Z_LSB | (Z_MSB << 8);

//			int x_data = (X_LSB);
//			int y_data = (Y_LSB);
//			int z_data = Z_LSB;
//
//			signed short x_data = (X_LSB - X_OFFSET)*x_scale;
//			signed short y_data = (Y_LSB - Y_OFFSET)*y_scale;
//			signed short z_data = Z_LSB;

//			int x_data = multiple_byte_array[1] - X_OFFSET;
//			int y_data = multiple_byte_array[3] - Y_OFFSET;
//			int z_data = Z_LSB;





			float rads = atan2(y_data,x_data);
//
			if(rads < 0)
			{
				rads+= 2*PI;
			}


			unsigned short deg = rads*180/PI;






			double reading = 0;

			double argument = x_data/y_data;
			double term2 = atan(argument)*180/PI;
//			double term2 = atan2(y_data,x_data)*180/PI;

			if(y_data > 0)
			{
				reading = 90 - term2;
			}

			else if(y_data <0)
			{
				reading = 270 - term2;
			}

			else if(y_data == 0 && x_data < 0)
			{
				reading = 180;
			}

			else if(y_data == 0 && x_data > 0)
			{
				reading = 0;
			}





	//		if between 345 and  15 deg
			if(deg < 345 && deg > 15)
			{
				GREEN_LED_ON();
			}
			else
				GREEN_LED_OFF();



			if(isTimerExpired == 1)
			{
				isTimerExpired = 0;
//				sprintf(LCDmessage, "%u",deg);
				snprintf(LCDmessage, 5,"%4d",(short) reading);
				SLCD_WriteMsg(LCDmessage);
			}
//		}
	}


}

void ACC_CAL(void)
{
	while(CompassState == ACCEL)
	{
		__disable_irq();

		SLCD_WriteMsg((unsigned char *)"ACAL");
	//	Check to make sure device id is correct
		int id = I2C_SingleByteRead(ACC_ADDR,ACC_ID_REG);
		if(id != ACC_ID)
		{
			SLCD_WriteMsg((unsigned char *)"ERROR");
		}

		int ctrl_reg1 = 0x2A;
		int LNOISE_on = 1 << 2;

		I2C_SingleByteWrite(ACC_ADDR,ctrl_reg1,LNOISE_on | 0x1);
		delay(100);


		// should be 4
		int sanity_check = I2C_SingleByteRead(ACC_ADDR,ctrl_reg1);

		int ctrl_reg2 = 0x2B;
		int high_res = 1 << 4;

	//	Turn on high resolution power mode
		I2C_SingleByteWrite(ACC_ADDR,ctrl_reg2,high_res);
		int sanity_check2 = I2C_SingleByteRead(ACC_ADDR,ctrl_reg2);

//		Ensure 2g mode is enabled pg 27
		I2C_SingleByteWrite(ACC_ADDR,0x0E, 0x0);
//		Enable High-pass filter
//		I2C_SingleByteWrite(ACC_ADDR,0x0E, 1 << 4);

		signed short X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB = 0 ;

		X_MSB = I2C_SingleByteRead(ACC_ADDR,ACC_X_MSB_REG);
		delay(10);
		X_LSB = I2C_SingleByteRead(ACC_ADDR,ACC_X_LSB_REG);
		delay(10);
		Y_MSB = I2C_SingleByteRead(ACC_ADDR,ACC_Y_MSB_REG);
		delay(10);
		Y_LSB = I2C_SingleByteRead(ACC_ADDR,ACC_Y_LSB_REG);
		delay(10);
		Z_MSB = I2C_SingleByteRead(ACC_ADDR,ACC_Z_MSB_REG);
		delay(10);
		Z_LSB = I2C_SingleByteRead(ACC_ADDR,ACC_Z_LSB_REG);
		delay(10);


	//		appending LSB to MSB to form 14-bit
		signed short x_data = ((X_MSB << 8) | (X_LSB)) >> 2;
		signed short y_data = ((Y_MSB << 8) | (Y_LSB)) >> 2;
		signed short z_data = ((Z_MSB << 8) | (Z_LSB)) >> 2;
	//	signed short x_data = X_LSB;
	//	signed short y_data = Y_LSB;
	//	signed short z_data = Z_LSB;
		X_ACCEL_OFFSET = x_data;
		Y_ACCEL_OFFSET = y_data;

//		sensitivity is 4096 counts/g for 2g mode pg 10
//		averaging
		for( int i = 0; i < 10000; i++)
		{
			gravity = (gravity + z_data)/2;


		}

		__enable_irq();
	}


}

void I2C_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[I2C_SDA] |= PORT_PCR_MUX(5);
	PORTE->PCR[I2C_SCL] |= PORT_PCR_MUX(5);

//	Pull up/down select
	PORTE->PCR[I2C_SDA] |= PORT_PCR_PS_MASK;
	PORTE->PCR[I2C_SCL] |= PORT_PCR_PS_MASK;

//	Enable pull up/down
	PORTE->PCR[I2C_SDA] |= PORT_PCR_PE_MASK;
	PORTE->PCR[I2C_SCL] |= PORT_PCR_PE_MASK;

//	Turn on the clock to I2C module
	SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;



//	Disable I2C module
	I2C0->C1 &= ~I2C_C1_IICEN_MASK;

//	COnfigure I2C baud rate for 100 KHz
	I2C0->F |= I2C_F_MULT(0x00);
	I2C0->F |= I2C_F_ICR(0x14);

//	Enable I2C module
	I2C0->C1 |= I2C_C1_IICEN_MASK;

}





void TPM0_IRQHandler(void)
{
// --------------------------------------------------------------------
// Place your timer ISR related code and register settings here - Start
	isTimerExpired = 1;
// pg 566 writes a 1 to timer overflow flag bit, which resets the flag to 0
//	bIsTimerExpired = true;
	TPM0->SC  |= TPM_SC_TOF_MASK;



// Place your timer ISR related code and register settings here - End
// --------------------------------------------------------------------
}

void PORTC_PORTD_IRQHandler(void)
{
// --------------------------------------------------------------------
// Place your port ISR related code and register settings here - Start

 	unsigned int sw1_pressed = PT_STATE(SW1) != PIN(SW1);
	unsigned int sw3_pressed = PT_STATE(SW3) != PIN(SW3);

	//	if switch 1 pressed (reset)
	if(sw1_pressed && CompassState == RUNNING)
	{

		//pg 194 WRiting logic 1 to the Interrupt Status Flag bit clears the register so we  can go back to main code
		PORTC->PCR[SW1] |= (PORT_PCR_ISF_MASK);

		CompassState = STOPPED;
	}

	//  if switch 3 pressed (pause/resume)
	else if(sw3_pressed)
	{

		//pg 194 WRiting logic 1 to the Interrupt Status Flag bit clears the register so we  can go back to main code
		PORTC->PCR[SW3] |= (PORT_PCR_ISF_MASK);

		switch(CompassState)
		{
			case STOPPED:
			{
				CompassState = ACQUIRING;
				break;
			}

			case RUNNING:
			{
				break;
			}

			case ACQUIRING:
			{
				CompassState = CALIBRATING;
				break;
			}

			case CALIBRATING:
			{
				CompassState = ACCEL;
				break;
			}

			case ACCEL:
			{
				CompassState = RUNNING;
				break;
			}

		}

	}



// Place your port ISR related code and register settings here - End
// --------------------------------------------------------------------
}




int main(void)
{
	__disable_irq();

	LED_Init();
	SWITCH_Init();
	TIMER_Init();
	SLCD_Init();
	I2C_Init();

	    NVIC_EnableIRQ(I2C0_IRQn);
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	NVIC_EnableIRQ(TPM0_IRQn);

	__enable_irq();

	int test = I2C_SingleByteRead(MAG_ADDR,MAG_X_MSB_REG);

	STOP();



	while(1)
	{
		switch(CompassState)
		{
			case STOPPED:
			{
				STOP();
				break;
			}
			case RUNNING:
			{
				RUN();
				break;
			}
			case ACQUIRING:
			{
				MAG_ACQ();
				break;
			}
			case CALIBRATING:
			{
				MAG_CAL();
				break;
			}

			case ACCEL:
			{
				ACC_CAL();
				break;
			}
		}

	}

    /* Never leave main */
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
