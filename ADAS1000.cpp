/***************************************************************************//**
 *   @file   		ADAS1000.cpp
 *   @brief  		Implementation of ADAS1000 Driver.
 *   @author 		ACozma (andrei.cozma@analog.com)
 *	 @adaptation	Mujtaba Jalil (mujtabaabduljalil@yahoo.com)
 *					Adapted the original 'C' code provided by Analog
 *					Devices to 'C++' for use with Arduino.
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 570
*******************************************************************************/

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include "ADAS1000.h"				
//#include <SPI.h>
#include <SPI.h>
#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif


/*****************************************************************************/
/************************ Variables Definitions ******************************/
/*****************************************************************************/
static unsigned long frameSize 		 = 0; //ADAS1000 frame size in bytes
static unsigned long frameRate 		 = 0; //ADAS1000 frame rate
static unsigned long inactiveWordsNo = 0; //number of inactive words in a frame

/***************************************************************************//**
 * @brief Initializes the SPI communication with ADAS1000 and checks if the 
 * 		  device is present by reading its ID. If the ID read is ok the ADAS1000
 *		  is configured with the spefified frame rate and all the words in a
 *		  frame are activated.
 *
 * @param rate - ADAS1000 frame rate.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
ADAS1000::ADAS1000()
{
}

unsigned char ADAS1000::Init(unsigned long rate)
{ 
	unsigned long minSpiFreq = 0;
	unsigned long revId 	 = 0;
    
	//store the selected frame rate
	frameRate = rate;
	//SPI.begin(); //SPI init command
	// Compute the SPI clock frequency.
	switch(frameRate)
	{
		case ADAS1000_16KHZ_FRAME_RATE:
			minSpiFreq = ADAS1000_16KHZ_FRAME_RATE * 
					  	 ADAS1000_16KHZ_WORD_SIZE * 
					  	 ADAS1000_16KHZ_FRAME_SIZE;
		break;
		case ADAS1000_128KHZ_FRAME_RATE:
			minSpiFreq = ADAS1000_128KHZ_FRAME_RATE * 
					  	 ADAS1000_128KHZ_WORD_SIZE * 
					  	 ADAS1000_128KHZ_FRAME_SIZE;
		break;
		case ADAS1000_31_25HZ_FRAME_RATE:
			minSpiFreq = (ADAS1000_31_25HZ_FRAME_RATE * 
					  	 ADAS1000_31_25HZ_WORD_SIZE * 
					  	 ADAS1000_31_25HZ_FRAME_SIZE) / 100;
		break;
		default: // ADAS1000_2KHZ__FRAME_RATE
			minSpiFreq = ADAS1000_2KHZ_FRAME_RATE * 
					  	 ADAS1000_2KHZ_WORD_SIZE * 
					  	 ADAS1000_2KHZ_FRAME_SIZE;
		break;
	}

	// Initialize the SPI controller. 
	// The SPI frequency must be greater or equal to minSpiFreq.
	/* if(!SPI_Init(0, minSpiFreq, 1, 1))
	{		
		return 0;
	} */

	// Reset the ADAS1000.
	SoftwareReset();

	// Activate all the channels
	inactiveWordsNo = 0;
	SetInactiveFrameWords(0);

	//Set the frame rate
	SetFrameRate(frameRate);

	return 1;
}

/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param regVal - Pointer to a variable where to store the read data.
 *
 * @return None.
*******************************************************************************/
void ADAS1000::GetRegisterValue(unsigned char regAddress, unsigned long* regVal)
{
	unsigned char readCmd[4]	= {0, 0, 0, 0};
	unsigned char readData[4]	= {0, 0, 0, 0};
	//unsigned long reg = 0;
	// Select the register (For register reads, data is shifted out
	// during the next word).
	readCmd[0] = regAddress;	// Register address.
	SPI_Write(readCmd, 4);
	
	// Read the data from the device.
	SPI_Read(readData, 4);
	*regVal = ((unsigned long)readData[1] << 16) +
			 ((unsigned long)readData[2] << 8) +
			 ((unsigned long)readData[3] << 0);
	//reg = *regVal;		 
	//Serial.println(reg);
	//delay(5000);
	return;
}

/***************************************************************************//**
 * @brief Writes a value to the selected register
 *
 * @param regAddress - The address of the register to write to.
 * @param regValue - The value to write to the register.
 *
 * @return None.    
*******************************************************************************/
void ADAS1000::SetRegisterValue(unsigned char regAddress,
                               unsigned long regVal)
{
	unsigned char writeCmd[4] = {0, 0, 0, 0};
	
	writeCmd[0] = 0x80 + regAddress;	// Write bit and register address.
	writeCmd[1] = (unsigned char)((regVal & 0xFF0000) >> 16);
	writeCmd[2] = (unsigned char)((regVal & 0x00FF00) >> 8);
	writeCmd[3] = (unsigned char)((regVal & 0x0000FF) >> 0);
	SPI_Write(writeCmd, 4);
}

/***************************************************************************//**
 * @brief Resets the ADAS1000 part.
 *
 * @return None.
*******************************************************************************/
void ADAS1000::SoftwareReset(void)
{
	
	unsigned long hell = 0;
	// Clear all registers to their reset value.
	//Serial.println(ADAS1000_ECGCTL_SWRST); // Just for debug purpose
	SetRegisterValue(ADAS1000_ECGCTL, ADAS1000_ECGCTL_SWRST);
	//GetRegisterValue(ADAS1000_ECGCTL, regVal);
	//Serial.println(hell); // Debug purpose

	// The software reset requires a NOP command to complete the reset.
	SetRegisterValue(ADAS1000_NOP, 0);
}

/***************************************************************************//**
 * @brief Selects which words are not included in a data frame.
 *
 * @param channelsMask - Specifies the words to be excluded from the data 
 * 						 frame using a bitwise or of the corresponding bits
 * 						 from the Frame Control Register.
 * 
 * @return None.
*******************************************************************************/
void ADAS1000::SetInactiveFrameWords(unsigned long wordsMask)
{
	unsigned long frmCtrlRegVal = 0;
	unsigned char i = 0;
	
	// Read the current value of the Frame Control Register
	GetRegisterValue(ADAS1000_FRMCTL, &frmCtrlRegVal);
	//Serial.println(frmCtrlRegVal);
	//set the inactive channles
	frmCtrlRegVal &= ~ADAS1000_FRMCTL_WORD_MASK;
	frmCtrlRegVal |= wordsMask;

	// Write the new value to the Frame Coontrol register.
	SetRegisterValue(ADAS1000_FRMCTL, frmCtrlRegVal);
	
	//compute the number of inactive words
	inactiveWordsNo = 0;
	for(i = 0; i < 32; i++)
	{
		if(wordsMask & 0x00000001ul)
		{
			inactiveWordsNo++;
		}
		wordsMask >>= 1;
	}
	
	//compute the new frame size
	switch(frameRate)
	{
		case ADAS1000_16KHZ_FRAME_RATE:
			frameSize = (ADAS1000_16KHZ_WORD_SIZE / 8) *
						(ADAS1000_16KHZ_FRAME_SIZE - inactiveWordsNo);
		break;
		case ADAS1000_128KHZ_FRAME_RATE:
			frameSize = (ADAS1000_128KHZ_WORD_SIZE / 8) *
						(ADAS1000_128KHZ_FRAME_SIZE - inactiveWordsNo);
		break;
		case ADAS1000_31_25HZ_FRAME_RATE:
			frameSize = ((ADAS1000_31_25HZ_WORD_SIZE / 8) *
						(ADAS1000_31_25HZ_FRAME_SIZE - inactiveWordsNo)) / 100;
		break;
		default: // ADAS1000_2KHZ__FRAME_RATE
			frameSize = (ADAS1000_2KHZ_WORD_SIZE / 8) *
						(ADAS1000_2KHZ_FRAME_SIZE - inactiveWordsNo);
		break;
	}
}

/***************************************************************************//**
 * @brief Sets the frame rate.
 *
 * @param rate - ADAS1000 frame rate.
 * 
 * @return None.
*******************************************************************************/
void ADAS1000::SetFrameRate(unsigned long rate)
{
	unsigned long frmCtrlRegVal = 0;
	
	// Store the selected frame rate
	frameRate = rate;
	
	// Read the current value of the Frame Control Register
	GetRegisterValue(ADAS1000_FRMCTL,&frmCtrlRegVal);
	frmCtrlRegVal &= ~ADAS1000_FRMCTL_FRMRATE_MASK;
	
	// Compute the new frame size and update the Frame Control Register value
	switch(frameRate)
	{
		case ADAS1000_16KHZ_FRAME_RATE:
			frameSize = (ADAS1000_16KHZ_WORD_SIZE / 8) *
						(ADAS1000_16KHZ_FRAME_SIZE - inactiveWordsNo);
			frmCtrlRegVal |= ADAS1000_FRMCTL_FRMRATE_16KHZ;
		break;
		case ADAS1000_128KHZ_FRAME_RATE:
			frameSize = (ADAS1000_128KHZ_WORD_SIZE / 8) *
						(ADAS1000_128KHZ_FRAME_SIZE - inactiveWordsNo);
			frmCtrlRegVal |= ADAS1000_FRMCTL_FRMRATE_128KHZ;
		break;
		case ADAS1000_31_25HZ_FRAME_RATE:
			frameSize = ((ADAS1000_31_25HZ_WORD_SIZE / 8) *
						(ADAS1000_31_25HZ_FRAME_SIZE - inactiveWordsNo)) / 100;
			frmCtrlRegVal |= ADAS1000_FRMCTL_FRMRATE_31_25HZ;
		break;
		default: // ADAS1000_2KHZ__FRAME_RATE
			frameSize = (ADAS1000_2KHZ_WORD_SIZE / 8) *
						(ADAS1000_2KHZ_FRAME_SIZE - inactiveWordsNo);
			frmCtrlRegVal |= ADAS1000_FRMCTL_FRMRATE_2KHZ;
		break;
	}
	
	// Write the new Frame control Register value
	SetRegisterValue(ADAS1000_FRMCTL, frmCtrlRegVal);
}

/***************************************************************************//**
 * @brief Reads the specified number of frames.
 *
 * @param pDataBuffer - Buffer to store the read data.
 * @param frameCnt - Number of frames to read.
 * @param startRead - Set to 1 if a the frames read sequence must be started.
 * @param stopRead - Set to 1 if a the frames read sequence must be sopped 
 *					 when exiting the function.
 * @param waitForReady - Set to 1 if the function must wait for the READY bit 
 *						 to be set in the header.
 * @param readyRepeat - Set to 1 if the device was configured to repeat the 
 *						header until the READY bit is set.
 *
 * @return  None.
*******************************************************************************/
void ADAS1000::ReadData(unsigned char* pDataBuffer, unsigned long frameCnt,
					   unsigned char startRead, unsigned char stopRead,
					   unsigned char waitForReady, unsigned char readyRepeat)
{
	unsigned char readCmd[4]	= {0, 0, 0, 0};
	unsigned long ready = 0;
	
	// If the read sequence must be started send a FRAMES command.
	if(startRead)
	{
		readCmd[0] = ADAS1000_FRAMES;	// Register address.
		SPI_Write(readCmd, 4);
	}

	// Read the number of requested frames.
	while(frameCnt)
	{
		// If waiting for the READY bit to be set read the header until the bit is set, otherwise just read the entire frame.
		if(waitForReady)
		{
			ready = 1;
			while(ready == 1)
			{
				//if the header is repeated until the READY bit is set read only the header, otherwise read the entire frame
				if(readyRepeat)
				{
					SPI_Read(pDataBuffer, 4);
					ready = *pDataBuffer & 0x40;
					//Serial.print(pDataBuffer[5]);//for debug
					if(ready == 0)
					{
						SPI_Read(pDataBuffer + 4, frameSize - 4);
						pDataBuffer += frameSize;
						frameCnt--;
						//Serial.print(pDataBuffer[5]);//for debug
					}
				}
				else
				{
					SPI_Read(pDataBuffer, frameSize);
					ready = *pDataBuffer & 0x40;
					if(ready == 0)
					{
						pDataBuffer += frameSize;			
						frameCnt--;
						//Serial.print(pDataBuffer[5]);//for debug
					}
				}
			}
		}
		else
		{
			SPI_Read(pDataBuffer, frameSize);
			pDataBuffer += frameSize;			
			frameCnt--;
		}
	}

	// If the frames read sequence must be stopped read a register to stop the frames read.
	if(stopRead)
	{
		GetRegisterValue(ADAS1000_FRMCTL,&ready);
		//Serial.print(pDataBuffer[5]);//for debug
	}
}

/***************************************************************************//**
 * @brief Computes the CRC for a frame.
 *
 * @param pBuf - Buffer holding the frame data.
 *
 * @return Returns the CRC value for the given frame.
*******************************************************************************/
unsigned long ADAS1000::ComputeFrameCrc(unsigned char *pBuf)
{
	unsigned char i = 0;
    unsigned long crc = 0xFFFFFFFFul;
	unsigned long poly = 0;
	unsigned char bitCnt = 0;	
	unsigned long frmSize = 0;
	
	// Select the CRC poly and word size based on the frame rate.
	if(frameRate == ADAS1000_128KHZ_FRAME_RATE)
	{
		poly = CRC_POLY_128KHZ;
		bitCnt = 16;
	}
	else
	{
		poly = CRC_POLY_2KHZ_16KHZ;
		bitCnt = 24;
	}

	frmSize = frameSize;

	// Compute the CRC.
	while(frmSize--)
	{
		crc ^= (((unsigned long)*pBuf++) << (bitCnt - 8));
		for(i = 0; i < 8; i++)
		{
			if(crc & (1ul << (bitCnt - 1)))
				crc = (crc << 1) ^ poly;
            else
				crc <<= 1;
		}
	}
	
	return crc;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param data - data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
/*******************************************************************************
 * Make sure to use A5 as the chip select
 *
 *
 *
 *
********************************************************************************/


unsigned char ADAS1000::SPI_Write(unsigned char* data,
						unsigned char bytesNumber)
{
	
	unsigned int i = 0;
	digitalWrite(A5,LOW); //A4 is a GPIO pin that is active low
	SPI.beginTransaction(SPISettings(5120000,MSBFIRST,SPI_MODE3));
	for (int i = 0; i < bytesNumber; i++)
	{
		SPI.transfer(data[i]);
	}
	digitalWrite(A5,HIGH);
	SPI.endTransaction();
	return bytesNumber;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - Data represents the read buffer.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
unsigned char ADAS1000::SPI_Read(unsigned char* data,
					   unsigned char bytesNumber)
{
	unsigned int i = 0;
	/* if(digitalRead(A3)==LOW)
	{
		digitalWrite(A4,LOW);
		SPI.beginTransaction(SPISettings(5120000,MSBFIRST,SPI_MODE3));
		for (int i = 0; i < bytesNumber; i++)
		{
			data[i] = SPI.transfer(0);
		}
		digitalWrite(A4,HIGH);
		SPI.endTransaction();
	} */
 	digitalWrite(A5,LOW);
	SPI.beginTransaction(SPISettings(5120000,MSBFIRST,SPI_MODE3));
	for (int i = 0; i < bytesNumber; i++)
	{
		data[i] = SPI.transfer(0);
	}
	digitalWrite(A5,HIGH);
	SPI.endTransaction();
	return bytesNumber; 
}
