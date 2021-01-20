/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
//#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum  State { stop, waiting, getRegisterAddress, getData, sendData , getsecondData,getwrd };
uint8_t i2ctransferState = stop;
enum PCD_Register {
	// Page 0: Command and status
	//						  0x00			// reserved for future use
	CommandReg				= 0x01 << 1,	// starts and stops command execution
	ComIEnReg				= 0x02 << 1,	// enable and disable interrupt request control bits
	DivIEnReg				= 0x03 << 1,	// enable and disable interrupt request control bits
	ComIrqReg				= 0x04 << 1,	// interrupt request bits
	DivIrqReg				= 0x05 << 1,	// interrupt request bits
	ErrorReg				= 0x06 << 1,	// error bits showing the error status of the last command executed
	Status1Reg				= 0x07 << 1,	// communication status bits
	Status2Reg				= 0x08 << 1,	// receiver and transmitter status bits
	FIFODataReg				= 0x09 << 1,	// input and output of 64 uint8_t FIFO buffer
	FIFOLevelReg			= 0x0A << 1,	// number of uint8_ts stored in the FIFO buffer
	WaterLevelReg			= 0x0B << 1,	// level for FIFO underflow and overflow warning
	ControlReg				= 0x0C << 1,	// miscellaneous control registers
	BitFramingReg			= 0x0D << 1,	// adjustments for bit-oriented frames
	CollReg					= 0x0E << 1,	// bit position of the first bit-collision detected on the RF interface
	//						  0x0F			// reserved for future use

	// Page 1:Command
	// 						  0x10			// reserved for future use
	ModeReg					= 0x11 << 1,	// defines general modes for transmitting and receiving
	TxModeReg				= 0x12 << 1,	// defines transmission data rate and framing
	RxModeReg				= 0x13 << 1,	// defines reception data rate and framing
	TxControlReg			= 0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
	TxASKReg				= 0x15 << 1,	// controls the setting of the transmission modulation
	TxSelReg				= 0x16 << 1,	// selects the internal sources for the antenna driver
	RxSelReg				= 0x17 << 1,	// selects internal receiver settings
	RxThresholdReg			= 0x18 << 1,	// selects thresholds for the bit decoder
	DemodReg				= 0x19 << 1,	// defines demodulator settings
	// 						  0x1A			// reserved for future use
	// 						  0x1B			// reserved for future use
	MfTxReg					= 0x1C << 1,	// controls some MIFARE communication transmit parameters
	MfRxReg					= 0x1D << 1,	// controls some MIFARE communication receive parameters
	// 						  0x1E			// reserved for future use
	SerialSpeedReg			= 0x1F << 1,	// selects the speed of the serial UART interface

	// Page 2: Configuration
	// 						0x20			// reserved for future use
	CRCResultRegH			= 0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
	CRCResultRegL			= 0x22 << 1,
	// 						  0x23			// reserved for future use
	ModWidthReg				= 0x24 << 1,	// controls the ModWidth setting?
	// 						  0x25			// reserved for future use
	RFCfgReg				= 0x26 << 1,	// configures the receiver gain
	GsNReg					= 0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
	CWGsPReg				= 0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
	ModGsPReg				= 0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
	TModeReg				= 0x2A << 1,	// defines settings for the internal timer
	TPrescalerReg			= 0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	TReloadRegH				= 0x2C << 1,	// defines the 16-bit timer reload value
	TReloadRegL				= 0x2D << 1,
	TCounterValueRegH		= 0x2E << 1,	// shows the 16-bit timer value
	TCounterValueRegL		= 0x2F << 1,

	// Page 3:Test Registers
	// 						  0x30			// reserved for future use
	TestSel1Reg				= 0x31 << 1,	// general test signal configuration
	TestSel2Reg				= 0x32 << 1,	// general test signal configuration
	TestPinEnReg			= 0x33 << 1,	// enables pin output driver on pins D1 to D7
	TestPinValueReg			= 0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
	TestBusReg				= 0x35 << 1,	// shows the status of the internal test bus
	AutoTestReg				= 0x36 << 1,	// controls the digital self test
	VersionReg				= 0x37 << 1,	// shows the software version
	AnalogTestReg			= 0x38 << 1,	// controls the pins AUX1 and AUX2
	TestDAC1Reg				= 0x39 << 1,	// defines the test value for TestDAC1
	TestDAC2Reg				= 0x3A << 1,	// defines the test value for TestDAC2
	TestADCReg				= 0x3B << 1		// shows the value of ADC I and Q channels
	// 						  0x3C			// reserved for production tests
	// 						  0x3D			// reserved for production tests
	// 						  0x3E			// reserved for production tests
	// 						  0x3F			// reserved for production tests
};

// MFRC522 comands. Described in chapter 10 of the datasheet.
enum PCD_Command {
	PCD_Idle				= 0x00,		// no action, cancels current command execution
	PCD_Mem					= 0x01,		// stores 25 uint8_ts into the internal buffer
	PCD_GenerateRandomID	= 0x02,		// generates a 10-uint8_t random ID number
	PCD_CalcCRC				= 0x03,		// activates the CRC coprocessor or performs a self test
	PCD_Transmit			= 0x04,		// transmits data from the FIFO buffer
	PCD_NoCmdChange			= 0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	PCD_Receive				= 0x08,		// activates the receiver circuits
	PCD_Transceive 			= 0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	PCD_MFAuthent 			= 0x0E,		// performs the MIFARE standard authentication as a reader
	PCD_SoftReset			= 0x0F		// resets the MFRC522
};

// Commands sent to the PICC.
enum PICC_Command {
	// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
	PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
	PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 1
	PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 1
	PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	// The commands used for MIFARE Classic (from http://www.nxp.com/documents/data_sheet/MF1S503x.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A    ;0x60=1100000
	PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
	PICC_CMD_MF_READ		= 0x30,		// Reads one 16 uint8_t block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 uint8_t block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
	PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
	PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 uint8_t page to the PICC.
};

// MIFARE constants that does not fit anywhere else
enum MIFARE_Misc {
	MF_ACK					= 0xA,		// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
	MF_KEY_SIZE				= 6			// A Mifare Crypto1 key is 6 uint8_ts.
};

// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
enum PICC_Type {
	PICC_TYPE_UNKNOWN		= 0,
	PICC_TYPE_ISO_14443_4	= 1,	// PICC compliant with ISO/IEC 14443-4
	PICC_TYPE_ISO_18092		= 2, 	// PICC compliant with ISO/IEC 18092 (NFC)
	PICC_TYPE_MIFARE_MINI	= 3,	// MIFARE Classic protocol, 320 uint8_ts
	PICC_TYPE_MIFARE_1K		= 4,	// MIFARE Classic protocol, 1KB
	PICC_TYPE_MIFARE_4K		= 5,	// MIFARE Classic protocol, 4KB
	PICC_TYPE_MIFARE_UL		= 6,	// MIFARE Ultralight or Ultralight C
	PICC_TYPE_MIFARE_PLUS	= 7,	// MIFARE Plus
	PICC_TYPE_TNP3XXX		= 8,	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	PICC_TYPE_NOT_COMPLETE	= 255	// SAK indicates UID is not complete.
};

// A struct used for passing the UID of a PICC.
typedef struct {
	uint8_t		size;			// Number of uint8_ts in the UID. 4, 7 or 10.
	uint8_t		uiduint8_t[10];
	uint8_t		sak;			// The SAK (Select acknowledge) uint8_t returned from the PICC after successful selection.
} Uid;

// A struct named MIFARE_Key used for passing a MIFARE Crypto1 key
typedef struct {
	uint8_t		keyuint8_t[MF_KEY_SIZE];//keyuint8_t[MF_KEY_SIZE] is an array with six uint8_ts; keys A and B are each 6 uint8_ts long (see defintion of MF_KEY_SIZE above)
} MIFARE_Key;

#define		STATUS_OK						    0	// success
#define		STATUS_ERROR						1 // error
#define		STATUS_COLLISION				2 // Collission detected
#define		STATUS_TIMEOUT					3 // Timeout in communication.
#define		STATUS_NO_ROOM					4 // A buffer is not big enough.
#define		STATUS_INTERNAL_ERROR		5	// Internal error in the code. Should not happen ;-)
#define		STATUS_INVALID					6 // Invalid argument.
#define		STATUS_CRC_WRONG				7	// The CRC_A does not match
#define		STATUS_MIFARE_NACK			8	// A MIFARE PICC responded with NA
/* Exported define ------------------------------------------------------------*/
/* Exported macro -------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
// Member variables
Uid uid;								// Used by PICC_ReadCardSerial().

// Size of the MFRC522 FIFO
static const uint8_t FIFO_SIZE = 64;		// The FIFO is 64 uint8_ts.
/* Exported function prototypes ----------------------------------------------*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PICC_BLOCK_SIZE 16
#define MFRC522_CS_LOW	HAL_GPIO_WritePin(SS_GPIO_Port,SS_Pin,GPIO_PIN_RESET)
#define MFRC522_CS_HIGH	HAL_GPIO_WritePin(SS_GPIO_Port,SS_Pin,GPIO_PIN_SET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t cdrbuf[16];
uint8_t isav;
uint8_t	in[PICC_BLOCK_SIZE+2];
uint8_t out[PICC_BLOCK_SIZE] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
MIFARE_Key keym = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t i2ctransmitBuffer = 0;
uint8_t i2creceiveBuffer = 0;
uint8_t i2cregisterAddress = 0;
uint8_t i2cnumber=0;
uint8_t i2corder=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void PCD_WriteRegister(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
									uint8_t value		///< The value to write.
								) {
		MFRC522_CS_LOW;				// Select slave
		uint8_t ad = reg & 0x7E;
		HAL_SPI_Transmit(&hspi1,&ad, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi1,&value, 1, HAL_MAX_DELAY);
		MFRC522_CS_HIGH;			// Release slave again
} // End PCD_WriteRegister()

/**
 * Writes a number of uint8_ts to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegisterMulti(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
									uint8_t count,		///< The number of uint8_ts to write to the register
									uint8_t *values	///< The values to write. uint8_t array.
								) {
	MFRC522_CS_LOW;		// Select slave
	uint8_t ad = reg & 0x7E;
	HAL_SPI_Transmit(&hspi1,&ad, 1, HAL_MAX_DELAY);
	for (uint8_t index = 0; index < count; index++) {
		HAL_SPI_Transmit(&hspi1,&values[index], 1, HAL_MAX_DELAY);
	}
	MFRC522_CS_HIGH;		// Release slave again
} // End PCD_WriteRegister()

/**
 * Reads a uint8_t from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t PCD_ReadRegister(	uint8_t reg	///< The register to read from. One of the PCD_Register enums.
								) {
	uint8_t txb = 0x80 | (reg & 0x7E);
	uint8_t dum = 0x00;							
	uint8_t value;
	MFRC522_CS_LOW;			// Select slave
	HAL_SPI_Transmit(&hspi1,&txb, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1,&dum,&value,1,HAL_MAX_DELAY);
	MFRC522_CS_HIGH;			// Release slave again
	return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of uint8_ts from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadRegisterMulti(	uint8_t reg,		///< The register to read from. One of the PCD_Register enums.
								uint8_t count,		///< The number of uint8_ts to read
								uint8_t *values,	///< uint8_t array to store the values in.
								uint8_t rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	if (count == 0) {
		return;
	}
	//printf("Reading %d uint8_ts from register.", count);
	uint8_t dum = 0x00;	
	uint8_t address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
	MFRC522_CS_LOW;		// Select slave
	count--;								// One read is performed outside of the loop
	HAL_SPI_Transmit(&hspi1,&address, 1, HAL_MAX_DELAY);
	while (index < count) {
		if (index == 0 && rxAlign) { // Only update bit positions rxAlign..7 in values[0]
			// Create bit mask for bit positions rxAlign..7
			uint8_t mask = 0;
			for (uint8_t i = rxAlign; i <= 7; i++) {
				mask |= (1 << i);
			}
			// Read value and tell that we want to read the same address again.
			uint8_t value ;
			HAL_SPI_TransmitReceive(&hspi1,&address,&value,1,HAL_MAX_DELAY);
			// Apply mask to both current value of values[0] and the new data in value.
			values[0] = (values[index] & ~mask) | (value & mask);
		}
		else { // Normal case
			HAL_SPI_TransmitReceive(&hspi1,&address,&values[index],1,HAL_MAX_DELAY);
		}
		index++;
	}
	HAL_SPI_TransmitReceive(&hspi1,&dum,&values[index],1,HAL_MAX_DELAY);
	MFRC522_CS_HIGH;			// Release slave again
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask	///< The bits to set.
									) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask	///< The bits to clear.
									  ) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()



uint8_t PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
								uint8_t length,	///< In: The number of uint8_ts to transfer.
								uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
					 ) {
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);					// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterMulti(FIFODataReg, length, data);		// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73탎.
	uint16_t i = 5000;
	uint8_t n;
	while (1) {
		n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq   reserved CRCIRq reserved reserved
		if (n & 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop calculating CRC for new content in the FIFO.

	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadRegister(CRCResultRegL);
	result[1] = PCD_ReadRegister(CRCResultRegH);
	return STATUS_OK;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////
void PCD_Reset() {
	PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74탎. Let us be generous: 50ms.
	HAL_Delay(50);
	// Wait for the PowerDown bit in CommandReg to be cleared
	while (PCD_ReadRegister(CommandReg) & (1<<4)) {
		// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
	}
} // End PCD_Reset()

void PCD_AntennaOn() {
	uint8_t value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()

void PCD_Init() {
	PCD_Reset();

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(TPrescalerReg, 0xA9);	// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25탎.
    PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(TReloadRegL, 0xE8);

	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)	
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()



/**
 * Transfers data to the MFRC522 FIFO, executes a commend, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
										uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
										uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
										uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
										uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
										uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
										uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
										uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
										bool checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
									 ) {
	uint8_t n, _validBits;
	unsigned int i;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming	= (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterMulti(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);			// Execute the command
	if (command == PCD_Transceive) 	{
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86탎.
	i = 100;
	while (1) {
		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq   HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {	// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl   CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = PCD_ReadRegister(FIFOLevelReg);						// Number of uint8_ts in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;												// Number of uint8_ts returned
		PCD_ReadRegisterMulti(FIFODataReg, n, backData, rxAlign);		// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;	// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) { // CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		n = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (n != STATUS_OK) {
			return n;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()
									 
uint8_t PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
									uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
									uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
									uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
									uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
									uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									bool checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()



/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PCD_MIFARE_Transceive(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
										uint8_t sendLen,		///< Number of uint8_ts in sendData.
										bool acceptTimeout	///< True => A timeout is also success
									) {
	uint8_t result;
	uint8_t cmdBuffer[18]; // We need room for 16 uint8_ts data and 2 uint8_ts CRC_A.

	// Sanity check
	if (sendData == NULL || sendLen > 16) {
		return STATUS_INVALID;
	}

	// Copy sendData[] to cmdBuffer[] and add CRC_A
	memcpy(cmdBuffer, sendData, sendLen);
	result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (result != STATUS_OK) {
		return result;
	}
	sendLen += 2;

	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	uint8_t cmdBufferSize = sizeof(cmdBuffer);
	uint8_t validBits = 0;
	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, false);
	if (acceptTimeout && result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result != STATUS_OK) {
		return result;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()
									
/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_TwoStepHelper(	uint8_t command,	///< The command to use
									uint8_t blockAddr,	///< The block (0-0xff) number.
									long data		///< The data to transfer in step 2
									) {
	uint8_t result;
	uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.

	// Step 1: Tell the PICC the command and block address
	cmdBuffer[0] = command;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2, false); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(	(uint8_t *)&data, 4, true); // Adds CRC_A and accept timeout as success.
	if (result != STATUS_OK) {
		return result;
	}

	return STATUS_OK;
} // End MIFARE_TwoStepHelper()
									

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
									uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
									uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
							   ) {
	uint8_t validBits;
	uint8_t status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 uint8_ts long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);			// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;										// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()
								 

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
							) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()
							


/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_RequestA(uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
							) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_RequestA()





/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 uint8_ts.
 * Only 4 uint8_ts can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID uint8_ts		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
							uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
						 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel	= 1;
	uint8_t result;
	uint8_t count;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uiduint8_t[] that is used in the current Cascade Level.
	char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
	uint8_t bufferUsed;				// The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	// 		uint8_t 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	// 		uint8_t 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete uint8_ts, Low nibble: Extra bits.
	// 		uint8_t 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	// 		uint8_t 3: UID-data
	// 		uint8_t 4: UID-data
	// 		uint8_t 5: UID-data
	// 		uint8_t 6: BCC					Block Check Character - XOR of uint8_ts 2-5
	//		uint8_t 7: CRC_A
	//		uint8_t 8: CRC_A
	// The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of uint8_ts 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	uint8_t2	uint8_t3	uint8_t4	uint8_t5
	//		========	=============	=====	=====	=====	=====
	//		 4 uint8_ts		1			uid0	uid1	uid2	uid3
	//		 7 uint8_ts		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 uint8_ts		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);			// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while ( ! uidComplete) {
		// Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 uint8_ts
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 uint8_ts
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uiduint8_t[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t uint8_tsToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of uint8_ts needed to represent the known bits for this level.
		if (uint8_tsToCopy) {
			uint8_t maxuint8_ts = useCascadeTag ? 3 : 4; // Max 4 uint8_ts in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (uint8_tsToCopy > maxuint8_ts) {
				uint8_tsToCopy = maxuint8_ts;
			}
			for (count = 0; count < uint8_tsToCopy; count++) {
				buffer[index++] = uid->uiduint8_t[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while ( ! selectDone) {
			// Find out how many bits and uint8_ts to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//printf("SELECT: currentLevelKnownBits=%d", currentLevelKnownBits);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
				// Calulate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//printf("ANTICOLLISION: currentLevelKnownBits=%d", currentLevelKnownBits);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole uint8_ts in the UID part.
				index			= 2 + count;					// Number of whole uint8_ts: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a seperate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				result = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (result & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while ( ! selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID uint8_ts from buffer[] to uid->uiduint8_t[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		uint8_tsToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < uint8_tsToCopy; count++) {
			uid->uiduint8_t[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) {		// SAK must be exactly 24 bits (1 uint8_t + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those uint8_ts are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while ( ! uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_HaltA() {
	uint8_t result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is an success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, false);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()




/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery. A key consists of 6 uint8_ts.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
uint8_t PCD_Authenticate(uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
								uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
								MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 uint8_ts)
								Uid *uid			///< Pointer to Uid struct. The first 4 uint8_ts of the UID is used.
								) {
	uint8_t waitIRq = 0x10;		// IdleIRq

	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key uint8_ts
		sendData[2+i] = key->keyuint8_t[i];
	}
	for (uint8_t i = 0; i < 4; i++) {				// The first 4 uint8_ts of the UID
		sendData[8+i] = uid->uiduint8_t[i];
	}

	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData), NULL, 0, 0, 0, false);
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void PCD_StopCrypto1() {
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved   MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Reads 16 uint8_ts (+ 2 uint8_ts CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 uint8_ts starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 uint8_ts because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_Read(	uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
							uint8_t *buffer,		///< The buffer to store the data in
							uint8_t *bufferSize	///< Buffer size, at least 18 uint8_ts. Also number of uint8_ts returned if STATUS_OK.
						) {
	uint8_t result;

	// Sanity check
	if (buffer == NULL || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()

/**
 * Writes 16 uint8_ts to the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight the opretaion is called "COMPATIBILITY WRITE".
 * Even though 16 uint8_ts are transferred to the Ultralight PICC, only the least significant 4 uint8_ts (uint8_ts 0 to 3)
 * are written to the specified address. It is recommended to set the remaining uint8_ts 04h to 0Fh to all logic 0.
 * *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_Write(	uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
							uint8_t *buffer,	///< The 16 uint8_ts to write to the PICC
							uint8_t bufferSize	///< Buffer size, must be at least 16 uint8_ts. Exactly 16 uint8_ts are written.
						) {
	uint8_t result;

	// Sanity check
	if (buffer == NULL || bufferSize < 16) {
		return STATUS_INVALID;
	}

	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(cmdBuffer, 2, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(	buffer, bufferSize, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	return STATUS_OK;
} // End MIFARE_Write()

/**
 * Writes a 4 uint8_t page to the active MIFARE Ultralight PICC.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_Ultralight_Write(	uint8_t page, 		///< The page (2-15) to write to.
										uint8_t *buffer,	///< The 4 uint8_ts to write to the PICC
										uint8_t bufferSize	///< Buffer size, must be at least 4 uint8_ts. Exactly 4 uint8_ts are written.
									) {
	uint8_t result;

	// Sanity check
	if (buffer == NULL || bufferSize < 4) {
		return STATUS_INVALID;
	}

	// Build commmand buffer
	uint8_t cmdBuffer[6];
	cmdBuffer[0] = PICC_CMD_UL_WRITE;
	cmdBuffer[1] = page;
	memcpy(&cmdBuffer[2], buffer, 4);

	// Perform the write
	result = PCD_MIFARE_Transceive(cmdBuffer, 6, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
} // End MIFARE_Ultralight_Write()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_Decrement(	uint8_t blockAddr, ///< The block (0-0xff) number.
								long delta		///< This number is subtracted from the value of block blockAddr.
							) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_Increment(	uint8_t blockAddr, ///< The block (0-0xff) number.
								long delta		///< This number is added to the value of block blockAddr.
							) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

/**
 * MIFARE Restore copies the value of the addressed block into a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_Restore(	uint8_t blockAddr ///< The block (0-0xff) number.
							) {
	// The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
	// Doing only a single step does not work, so I chose to transfer 0L in step two.
	return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
} // End MIFARE_Restore()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t MIFARE_Transfer(	uint8_t blockAddr ///< The block (0-0xff) number.
							) {
	uint8_t result;
	uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.

	// Tell the PICC we want to transfer the result into block blockAddr.
	cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
} // End MIFARE_Transfer()


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////


/**
 * Returns a string pointer to a status code name.
 *
 */
const char *GetStatusCodeName(uint8_t code	///< One of the StatusCode enums.
										) {
	switch (code) {
		case STATUS_OK:							return "Success.";                                       break;
		case STATUS_ERROR:					return "Error in communication.";                        break;
		case STATUS_COLLISION:			return "Collission detected."; 													 break;
		case STATUS_TIMEOUT:				return "Timeout in communication."; 									   break;
		case STATUS_NO_ROOM:				return "A buffer is not big enough."; 									 break;
		case STATUS_INTERNAL_ERROR:	return "Internal error in the code. Should not happen."; break;
		case STATUS_INVALID:				return "Invalid argument.";                              break;
		case STATUS_CRC_WRONG:			return "The CRC_A does not match.";                      break;
		case STATUS_MIFARE_NACK:		return "A MIFARE PICC responded with NAK.";              break;
		default:										return "Unknown error";                                  break;
	}
} // End GetStatusCodeName()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
uint8_t PICC_GetType(uint8_t sak		///< The SAK uint8_t returned from PICC_Select().
							) {
	if (sak & 0x04) { // UID not complete
		return PICC_TYPE_NOT_COMPLETE;
	}

	switch (sak) {
		case 0x09:	return PICC_TYPE_MIFARE_MINI;	break;
		case 0x08:	return PICC_TYPE_MIFARE_1K;		break;
		case 0x18:	return PICC_TYPE_MIFARE_4K;		break;
		case 0x00:	return PICC_TYPE_MIFARE_UL;		break;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;	break;
		case 0x01:	return PICC_TYPE_TNP3XXX;		break;
		default:	break;
	}

	if (sak & 0x20) {
		return PICC_TYPE_ISO_14443_4;
	}

	if (sak & 0x40) {
		return PICC_TYPE_ISO_18092;
	}

	return PICC_TYPE_UNKNOWN;
} // End PICC_GetType()

/**
 * Returns a string pointer to the PICC type name.
 *
 */
const char *PICC_GetTypeName(uint8_t piccType	///< One of the PICC_Type enums.
										) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:			return "PICC compliant with ISO/IEC 14443-4";			break;
		case PICC_TYPE_ISO_18092:				return "PICC compliant with ISO/IEC 18092 (NFC)";	break;
		case PICC_TYPE_MIFARE_MINI:			return "MIFARE Mini, 320 uint8_ts";								break;
		case PICC_TYPE_MIFARE_1K:				return "MIFARE 1KB";															break;
		case PICC_TYPE_MIFARE_4K:				return "MIFARE 4KB";															break;
		case PICC_TYPE_MIFARE_UL:				return "MIFARE Ultralight or Ultralight C";				break;
		case PICC_TYPE_MIFARE_PLUS:			return "MIFARE Plus";															break;
		case PICC_TYPE_TNP3XXX:					return "MIFARE TNP3XXX";													break;
		case PICC_TYPE_NOT_COMPLETE:		return "SAK indicates UID is not complete.";			break;
		case PICC_TYPE_UNKNOWN:
		default:												return "Unknown type";														break;
	}
} // End PICC_GetTypeName()


/**
 * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tupples C1 is MSB (=4) and C3 is LSB (=1).
 */
void MIFARE_SetAccessBits(	uint8_t *accessBitBuffer,	///< Pointer to uint8_t 6, 7 and 8 in the sector trailer. uint8_ts [0..2] will be set.
									uint8_t g0,				///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
									uint8_t g1,				///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
									uint8_t g2,				///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
									uint8_t g3					///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
								) {
	uint8_t c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
	uint8_t c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
	uint8_t c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);

	accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
	accessBitBuffer[1] =          c1 << 4 | (~c3 & 0xF);
	accessBitBuffer[2] =          c3 << 4 | c2;
} // End MIFARE_SetAccessBits()

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return bool
 */
bool PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	uint8_t result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return bool
 */
bool PICC_ReadCardSerial() {
	uint8_t result = PICC_Select(&uid, 0);
	return (result == STATUS_OK);//return a '1' if PICC_Select returns a '1', else a '0'
} // End PICC_ReadCardSerial()

int PICC_WriteBlock(int blockNumber, uint8_t arrayAddress[], MIFARE_Key *key)
{
  //this makes sure that we only write into data blocks. Every 4th block is a trailer block for the access/security info.
  int largestModulo4Number=blockNumber/4*4;
  int trailerBlock=largestModulo4Number+3;//determine trailer block for the sector
  if (blockNumber > 2 && (blockNumber+1)%4 == 0){
	  return 2;
  }//block number is a trailer block (modulo 4); quit and send error code 2
  //printf("%d is a data block:", blockNumber);

  /*****************************************authentication of the desired block for access***********************************************************/
  uint8_t status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, key, &(uid));
  //uint8_t PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, Uid *uid);
  //this method is used to authenticate a certain block for writing or reading
  //command: See enumerations above -> PICC_CMD_MF_AUTH_KEY_A	= 0x60 (=1100000),		// this command performs authentication with Key A
  //blockAddr is the number of the block from 0 to 15.
  //MIFARE_Key *key is a pointer to the MIFARE_Key struct defined above, this struct needs to be defined for each block. New cards have all A/B= FF FF FF FF FF FF
  //Uid *uid is a pointer to the UID struct that contains the user ID of the card.
  if (status != STATUS_OK) {
         return 3;//return "3" as error message
  }
  //it appears the authentication needs to be made before every block read/write within a specific sector.
  //If a different sector is being authenticated access to the previous one is lost.


  /*****************************************writing the block***********************************************************/

  status = MIFARE_Write(blockNumber, arrayAddress, 16);//valueBlockA is the block number, MIFARE_Write(block number (0-15), uint8_t array containing 16 values, number of uint8_ts in block (=16))
  //status = mfrc522.MIFARE_Write(9, value1Block, 16);
  if (status != STATUS_OK) {
           return 4;//return "4" as error message
  }
  //printf("block was written\r\n");
  return status;
}


int PICC_ReadBlock(int blockNumber, uint8_t arrayAddress[], MIFARE_Key *key)
{
  int largestModulo4Number=blockNumber/4*4;
  int trailerBlock=largestModulo4Number+3;//determine trailer block for the sector

  /*****************************************authentication of the desired block for access***********************************************************/
  uint8_t status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, key, &(uid));
  //uint8_t PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, Uid *uid);
  //this method is used to authenticate a certain block for writing or reading
  //command: See enumerations above -> PICC_CMD_MF_AUTH_KEY_A	= 0x60 (=1100000),		// this command performs authentication with Key A
  //blockAddr is the number of the block from 0 to 15.
  //MIFARE_Key *key is a pointer to the MIFARE_Key struct defined above, this struct needs to be defined for each block. New cards have all A/B= FF FF FF FF FF FF
  //Uid *uid is a pointer to the UID struct that contains the user ID of the card.
  if (status != STATUS_OK) {
         return 3;//return "3" as error message
  }
  //it appears the authentication needs to be made before every block read/write within a specific sector.
  //If a different sector is being authenticated access to the previous one is lost.


  /*****************************************reading a block***********************************************************/

  uint8_t buffersize = 18;//we need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size...
  status = MIFARE_Read(blockNumber, arrayAddress, &buffersize);//&buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number
  if (status != STATUS_OK) {
          return 4;//return "4" as error message
  }
  //printf("block was read\r\n");
  return status;
}


bool MIFARE_OpenUidBackdoor() {
	// Magic sequence:
	// > 50 00 57 CD (HALT + CRC)
	// > 40 (7 bits only)
	// < A (4 bits only)
	// > 43
	// < A (4 bits only)
	// Then you can write to sector 0 without authenticating
	
	PICC_HaltA(); // 50 00 57 CD
	
	uint8_t cmd = 0x40;
	uint8_t validBits = 7; /* Our command is only 7 bits. After receiving card response,
						  this will contain amount of valid response bits. */
	uint8_t response[32]; // Card's response is written here
	uint8_t received;
	uint8_t status = PCD_TransceiveData(&cmd, (uint8_t)1, response, &received, &validBits, (uint8_t)0, false); // 40
	if(status != STATUS_OK) {
		return false;
	}
	if (received != 1 || response[0] != 0x0A) {
		return false;
	}
	
	cmd = 0x43;
	validBits = 8;
	status = PCD_TransceiveData(&cmd, (uint8_t)1, response, &received, &validBits, (uint8_t)0, false); // 43
	if(status != STATUS_OK) {
		return false;
	}
	if (received != 1 || response[0] != 0x0A) {
		return false;
	}
	
	// You can now write to sector 0 without authenticating!
	return true;
} // End MIFARE_OpenUidBackdoor()


bool MIFARE_SetUid(uint8_t *newUid, uint8_t uidSize,MIFARE_Key *key) {
	
	// UID + BCC uint8_t can not be larger than 16 together
	if (!newUid || !uidSize || uidSize > 15) {
		return false;
	}
	
	// Authenticate for reading
uint8_t status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, (uint8_t)1, key, &uid);
	if (status != STATUS_OK) {
		
		if (status == STATUS_TIMEOUT) {
			// We get a read timeout if no card is selected yet, so let's select one
			
			// Wake the card up again if sleeping
//			  uint8_t atqa_answer[2];
//			  uint8_t atqa_size = 2;
//			  PICC_WakeupA(atqa_answer, &atqa_size);
			
			if (!PICC_IsNewCardPresent() || !PICC_ReadCardSerial()) {
				return false;
			}
			
			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, (uint8_t)1, key, &uid);
			if (status != STATUS_OK) {
				// We tried, time to give up
				return false;
			}
		}
		else {
			return false;
		}
	}
	
	// Read block 0
	uint8_t block0_buffer[18];
	uint8_t uint8_tCount = sizeof(block0_buffer);
	status = MIFARE_Read((uint8_t)0, block0_buffer, &uint8_tCount);
	if (status != STATUS_OK) {
		return false;
	}
	
	// Write new UID to the data we just read, and calculate BCC uint8_t
	uint8_t bcc = 0;
	for (uint8_t i = 0; i < uidSize; i++) {
		block0_buffer[i] = newUid[i];
		bcc ^= newUid[i];
	}
	
	// Write BCC uint8_t to buffer
	block0_buffer[uidSize] = bcc;
	
	// Stop encrypted traffic so we can send raw uint8_ts
	PCD_StopCrypto1();
	
	// Activate UID backdoor
	if (!MIFARE_OpenUidBackdoor()) {
		return false;
	}
	
	// Write modified block 0 back to card
	status = MIFARE_Write((uint8_t)0, block0_buffer, (uint8_t)16);
	if (status != STATUS_OK) {
		return false;
	}
	
	// Wake the card up again
	uint8_t atqa_answer[2];
	uint8_t atqa_size = 2;
	PICC_WakeupA(atqa_answer, &atqa_size);
	
	return true;
}

bool MIFARE_UnbrickUidSector() {
	MIFARE_OpenUidBackdoor();
	
	uint8_t block0_buffer[] = {0x01, 0x02, 0x03, 0x04, 0x04, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	
	
	// Write modified block 0 back to card
	uint8_t status = MIFARE_Write((uint8_t)0, block0_buffer, (uint8_t)16);
	if (status != STATUS_OK) {
		return false;
	}
	return true;
}



void registerReadEvent (uint16_t address,uint8_t* data)
{
		switch (address)
  {
  	case 0: 
			*data = isav;
  		break;
  	case 1:
			*data = uid.uiduint8_t[i2cnumber];
  		break;
		case 2:
			PICC_ReadCardSerial();
			PICC_ReadBlock(i2cnumber, in, &keym);
			*data = in[i2corder];		
  		break;
  }
}
void registerWriteEvent(uint16_t address, uint8_t data,uint8_t order,uint8_t wrda)
{
	PICC_ReadCardSerial();
	PICC_ReadBlock(data, in, &keym);
	for(uint8_t blockcuff = 0;blockcuff < 16;blockcuff++) out[blockcuff] = in[blockcuff];
	if(address == 3)
		{
			out[order] = wrda;
			i2ctransmitBuffer = PICC_WriteBlock(data, out,&keym);
		}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  //MX_USART1_UART_Init();
	
  /* USER CODE BEGIN 2 */
	PCD_Init();
	HAL_Delay(100);
	PCD_Init();
	HAL_I2C_EnableListen_IT(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		isav =PICC_IsNewCardPresent(); 
		if(isav) 
		{
			PICC_ReadCardSerial();

			PICC_ReadBlock(i2cnumber, in, &keym);
			for(uint8_t blockcuff = 0;blockcuff < 16;blockcuff++) out[blockcuff] = in[blockcuff];
			if(i2cregisterAddress == 3)
			{
				out[i2corder] = i2creceiveBuffer;
				i2ctransmitBuffer = PICC_WriteBlock(i2cnumber, out,&keym);
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t direction, uint16_t addrMatchCode) { 
	switch (direction) {
		case I2C_DIRECTION_TRANSMIT:

			i2ctransferState = getRegisterAddress;
			if (HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME) != HAL_OK) {
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME);
			}
		break;
 
		case I2C_DIRECTION_RECEIVE:
			i2ctransferState = sendData;
 
			registerReadEvent(i2cregisterAddress,&i2ctransmitBuffer);
			if (HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, &i2ctransmitBuffer, 1, I2C_LAST_FRAME) != HAL_OK) {
			HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, &i2ctransmitBuffer, 1, I2C_LAST_FRAME);
			}
		break;
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	switch (i2ctransferState) {
		case getRegisterAddress:
			i2cregisterAddress = i2creceiveBuffer;
			i2ctransferState = getData;
			if (HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME) != HAL_OK) {
				HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME);
			}
		break;
 
		case getData:
		  i2cnumber = i2creceiveBuffer;
		  i2ctransferState = getsecondData;
			if (HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME) != HAL_OK) {
				HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME);
			}
		break;
			
		case getsecondData:
			i2corder = i2creceiveBuffer;
		  i2ctransferState = getwrd;
			if (HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME) != HAL_OK) {
				HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME);
			}
			break;
		
		case getwrd:
			registerWriteEvent(i2cregisterAddress,i2cnumber,i2corder,i2creceiveBuffer);
			if (HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME) != HAL_OK) {
				HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &i2creceiveBuffer, 1, I2C_FIRST_FRAME);
			}
			break;
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) { 
	HAL_I2C_EnableListen_IT(hi2c); // Restart
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
