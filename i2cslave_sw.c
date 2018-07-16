/*
 * i2cslave.c
 *
 *  Created on: 23.03.2015
 *      Author: dennis
 */

#include "config.h"
#ifdef USE_I2C_SW_DD

#include "i2cslave_sw.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Interrupt vector for SDA pin
#ifdef __AVR_ATmega644__
#define SDA_vector PCINT2_vect
#endif
#ifdef __AVR_ATtiny861__
#define SDA_vector PCINT_vect
#endif

// Dedicated general purpose registers.
register unsigned char _TWSR asm("r2");
register unsigned char _TWDR asm("r3");
register unsigned char _TWEA asm("r4");

#define DEBUG_HI() (DEV_LED_PORT |= _BV(DEV_LED_PIN))
#define DEBUG_LO() (DEV_LED_PORT &= ~_BV(DEV_LED_PIN))

//SDA and SCL bit in PORT variable must be 0!

// set SDA as input pin / tristate
#define TRISTATE_SDA() ( DEV_SDA_DDR &= ~_BV(DEV_SDA_OPIN) )
// set SCL as input pin / tristate
#define TRISTATE_SCL() ( DEV_SCL_DDR &= ~_BV(DEV_SCL_OPIN) )
// Set SDA to LO
#define ACK() ( DEV_SDA_DDR |= _BV(DEV_SDA_OPIN) )
// Set SCL to LO (for clock stretching)
#define SCL_LO() ( DEV_SCL_DDR |= _BV(DEV_SCL_OPIN) )
// Set SDA to LO
#define SDA_LO() ( DEV_SDA_DDR |= _BV(DEV_SDA_OPIN) )
// Read SDA pin and return: 0 for LO and 1 for HI. Note that is_bit_clear() / is_bit_set()
// from AVRLib does NOT return always 1 for HI.
#define READ_SDA() ( (DEV_SDA_IPORT & _BV(DEV_SDA_IPIN)? 1 : 0) )
// Read SCL pin and return: 0 for LO and 1 for HI. Note that is_bit_clear() / is_bit_set()
// from AVRLib does NOT return always 1 for HI.
#define READ_SCL() ( (DEV_SCL_IPORT & _BV(DEV_SCL_IPIN)? 1 : 0) )

/* External interrupt macros. These are device dependent. */
#ifdef __AVR_ATmega644__
#define INITIALIZE_TWI_INTERRUPT()
#define CLEAR_TWI_INTERRUPT_FLAG() (PCIFR = _BV(PCIF2))
// Enables SDA interrupt.
#define ENABLE_TWI_INTERRUPT() (PCICR |= _BV(PCIE2))
// Disables SDA interrupt.
#define DISABLE_TWI_INTERRUPT() (PCICR &= ~_BV(PCIE2))
#endif
#ifdef __AVR_ATtiny861__
// Sets falling edge of SDA generates interrupt.
#define INITIALIZE_TWI_INTERRUPT()
// Clears interrupt flag.
#define CLEAR_TWI_INTERRUPT_FLAG() (GIFR = _BV(PCIF))
// Enables SDA interrupt.
#define ENABLE_TWI_INTERRUPT() (GIMSK |= _BV(PCIE1))
// Disables SDA interrupt.
#define DISABLE_TWI_INTERRUPT() (GIMSK &= ~_BV(PCIE1))
#endif

/** name macros for twi state machine */
# define TWI_SLA_REQ_W_ACK_RTD 0x60
# define TWI_SLA_DATA_RCV_ACK_RTD 0x80
# define TWI_SLA_DATA_RCV_NACK_RTD 0x88
# define TWI_SLA_REQ_R_ACK_RTD 0xA8
# define TWI_SLA_DATA_SND_ACK_RCV 0xB8
# define TWI_SLA_DATA_SND_NACK_RCV 0xC0
# define TWI_SLA_LAST_DATA_SND_ACK_RCV 0xC8
# define TWI_SLA_REPEAT_START 0xA0
# define TWI_SLA_STOP 0x68
# define I2C_IDLE 0x00

/** size of incoming buffer */
#define IN_BUFFER_SIZE 32
/** size of outgoing buffer */
#define OUT_BUFFER_SIZE 32

/** Incoming buffer */
volatile uint8_t incomingBuffer[IN_BUFFER_SIZE];
/** Outgoing buffer */
volatile uint8_t outgoingBuffer[OUT_BUFFER_SIZE];
/** outgoing buffer content size */
uint8_t outgoingBufferSize = 0;

/** index into incoming buffer */
volatile uint8_t iwalker = 0;
/** index into outgoing buffer */
//volatile uint8_t owalker = 0;
/**
 * TWI slave init
 */
void twi_slave_init(uint8_t address) {
	// SCL as input
	TRISTATE_SCL();
	// SDA as input
	TRISTATE_SDA();
	// init interrupt
	INITIALIZE_TWI_INTERRUPT();
	// init Enable ACK register
	_TWEA = 1;
	// init state register
	_TWSR = I2C_IDLE;
}

/**
 * Enable TWI slave
 */
void twi_slave_enable(void) {
#ifdef __AVR_ATmega644__
	PCMSK2 |= _BV(PCINT17); // DD
#endif
#ifdef __AVR_ATtiny861__
	PCMSK0 |= _BV(PCINT0);
#endif
	CLEAR_TWI_INTERRUPT_FLAG();
	ENABLE_TWI_INTERRUPT();
}

/**
 * Disable TWI slave
 */
void twi_slave_disable(void) {
#ifdef __AVR_ATmega644__
	PCMSK2 &= ~_BV(PCINT17); // DD
#endif
#ifdef __AVR_ATtiny861__
	PCMSK0 &= ~_BV(PCINT0);
#endif
	DISABLE_TWI_INTERRUPT();
	CLEAR_TWI_INTERRUPT_FLAG();
}

/**
 * Write some data to outgoing buffer. Data in outgoing buffer will be send
 * later on if a master requests data.
 */
uint8_t twi_writeData(uint8_t *data, uint8_t size) {
	uint8_t i;
	for (i = 0; ((i < size) && (i < IN_BUFFER_SIZE)); i++) {
		outgoingBuffer[i] = data[i];
	}
	outgoingBufferSize = i;
	return i;
}

/*-------------------------*/
/** locally used functions, partially inline functions */
unsigned char receiveStartByte(void);
void receiveByte(void);
uint8_t checkStartCondition(void);
void TWI_state_machine(void);
void sendByte(void);
uint8_t calcBufferAddress(uint8_t address);

/*
 * ISR for negative edge on SDA
 *
 */
ISR(SDA_vector) {
	if (bit_is_set(DEV_SDA_IPORT, DEV_SDA_IPIN)) {
		// not a negative edge, simply return
		return;
	}
	if (_TWSR == I2C_IDLE) {
		// Check for START condition by checking SCL HI when SDA went LO
		if (bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN)) {
			// looks like START
			DISABLE_TWI_INTERRUPT();
		} else {
			// false trigger, exit ISR and reenable interrupt
			CLEAR_TWI_INTERRUPT_FLAG();
			ENABLE_TWI_INTERRUPT();
			return;
		}
		// check if START is for our slave address
		if (checkStartCondition()) {
			// if not, simply return
			return;
		}
	}
	// Call the TWI state machine
	TWI_state_machine();
	CLEAR_TWI_INTERRUPT_FLAG();
	ENABLE_TWI_INTERRUPT();
}

/**
 * Check for START condition and determine if we are addressed.
 * Returns 0 if we are addressed, 1 otherwise
 */
inline uint8_t checkStartCondition(void) {
	unsigned char startByte = 0;
	// loop for one or several start conditions before a STOP
	if (_TWSR == I2C_IDLE || _TWSR == TWI_SLA_REPEAT_START) {
		// read start byte and check address sent
		startByte = receiveStartByte();
		if (startByte == 0) {
			// STOP or foreign address received
			_TWSR = I2C_IDLE;
			CLEAR_TWI_INTERRUPT_FLAG();
			ENABLE_TWI_INTERRUPT();
			return 1;
		} else {
			// START condition and our own address
			if (startByte & 1)
				_TWSR = TWI_SLA_REQ_R_ACK_RTD;
			else {
				_TWSR = TWI_SLA_REQ_W_ACK_RTD;
			}
		}
	}
	_TWDR = startByte;
	return 0;
}

/**
 * Read start byte. This byte is send after a START condition. It contains
 * the slave address and READ/WRITE flag.
 * Check the address sent if this is our own address.
 *
 */
inline unsigned char receiveStartByte(void) {
	uint8_t i = 0;
	uint8_t byteValue = 0;
	uint8_t bitValue = 0;

	// loop until SCL goes LO
	loop_until_bit_is_clear(DEV_SCL_IPORT, DEV_SCL_IPIN);

	// read in 8 bits
	for (i = 0; i < 8; i++) {
		// loop until SCL is HI
		loop_until_bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN);
		// read in SDA
		bitValue = READ_SDA();
		// add bit value to byte value
		byteValue = (byteValue << 1) | bitValue;

		// loop until SCL is LO
		//loop_until_bit_is_clear(DEV_SCL_IPORT, DEV_SCL_IPIN);

		// We assume "standard" receive of byte and loop until SCL is LO
		// But also check in the loop for START/STOP condition, then this is not
		// a byte coming in but a START/STOP condition.
		while (bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN)) {
			if ((byteValue & 1) != bitValue) {
				// if SDA changes while SCL is HI:
				// - SDA goes HI: we have STOP condition
				// - SDA goes LO: we have (repeated) START condition.
				if (bit_is_set(DEV_SDA_IPORT, DEV_SDA_IPIN)) {
					_TWSR = TWI_SLA_STOP;
				} else {
					_TWSR = TWI_SLA_REPEAT_START;
				}
				return 0;
			} else {
				// get SDA bit value to check
				bitValue = bit_is_set(DEV_SDA_IPORT, DEV_SDA_IPIN);
			}
		}
	}

	if ((byteValue & 0xFE) == (SLAVE_ADDRESS << 1)) {
		// This slave is addressed, send ACK
		ACK();
		// keep ACK LO during complete next SCL HI period
		loop_until_bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN);
		loop_until_bit_is_clear(DEV_SCL_IPORT, DEV_SCL_IPIN);
		// set SDA as input
		TRISTATE_SDA();
#ifdef USE_CLOCK_STRETCHING
		// clock_stretching start, see receiveByte() and sendByte() for clock stretching stop
		// Note: enabling this fails on an ATtiny861 with 1MHz, so I commented it out.
		// But: On an ATmega644 with 16Mhz, its ok to use clock-stretching.
		SCL_LO();
#endif
	} else {
		_TWSR = I2C_IDLE;
		return 0;
	}
	return byteValue;
}

/**
 * TWI slave receive data (into _TWDR).
 * This function tries to receive a byte but does also detect a STOP/START condition.
 */
inline void receiveByte(void) {
	unsigned char i;
	uint8_t bitValue = 0;
	_TWDR = 0;

	// clock stretching stop, see readStartByte() for start clock stretching
	TRISTATE_SCL();

	// Read 8 bits from master, respond with ACK.
	// SCL could be high or low depending on CPU speed
	for (i = 0; i < 8; i++) {
		// loop until SCL is HI
		loop_until_bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN);
		// read in SDA
		bitValue = READ_SDA();
		// add bit value to Data Register
		_TWDR = (_TWDR << 1) | bitValue;

		// We assume "standard" receive of byte and loop until SCL is LO
		// But also check in the loop for START/STOP condition, then this is not
		// a byte coming in but a START/STOP condition.
		while (READ_SCL()) {
			//!if SDA changes while SCL is high, it indicates STOP or START
			// retest that SCL is HI because it may have become LO since READ_SCL() call...
			if ((_TWDR & 1) != READ_SDA()
					&& (DEV_SCL_IPORT & _BV(DEV_SCL_IPIN))) {
				if (READ_SDA()) {
					_TWSR = TWI_SLA_STOP;
				} else {
					_TWSR = TWI_SLA_REPEAT_START;
				}
				return;
			}
		}
	}
	if (_TWEA) {
		// Send ACK
		ACK();
		loop_until_bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN);
		loop_until_bit_is_clear(DEV_SCL_IPORT, DEV_SCL_IPIN);
		// set SDA as input
		TRISTATE_SDA();
		_TWSR = TWI_SLA_DATA_RCV_ACK_RTD;
		_TWEA = 0;
	} else {
		// Send no ACK
		_TWSR = TWI_SLA_DATA_RCV_NACK_RTD;
	}
}

/****************************************************************************************
 * TWI WRITE
 ****************************************************************************************/

/**
 * TWI slave send data (from _TWDR)
 *
 */
inline void sendByte(void) {
	uint8_t i;
	uint8_t bitValue = 0;

	// clock stretching stop, see readStartByte() for start clock stretching
	TRISTATE_SCL();

	for (i = 0; i < 8; i++) {
		// loop until SCL is LO
		loop_until_bit_is_clear(DEV_SCL_IPORT, DEV_SCL_IPIN);
		// get bit value of byte to send
		bitValue = ((_TWDR >> (7 - i)) & 1);
		if (bitValue) {
			TRISTATE_SDA();
		} else {
			SDA_LO();
		}
		// keep SDA stable during positive part of SCL
		// loop until SCL is HI
		loop_until_bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN);
	}
	// last bit: loop until SCL is LO
	loop_until_bit_is_clear(DEV_SCL_IPORT, DEV_SCL_IPIN);

	// set SDA as input
	TRISTATE_SDA();
	// loop until SCL is HI
	loop_until_bit_is_set(DEV_SCL_IPORT, DEV_SCL_IPIN);
	// read in SDA
	bitValue = bit_is_clear(DEV_SDA_IPORT, DEV_SDA_IPIN);
	// LO means Master sends ACK
	if (bitValue) {
		// got ACK
		_TWSR = TWI_SLA_DATA_SND_ACK_RCV;
	} else {
		// got NACK
		_TWSR = TWI_SLA_DATA_SND_NACK_RCV;
	}
}

/**
 * TWI state machine software algorithm that emulates the hardware TWI state machine.
 *
 */
void TWI_state_machine(void) {
	uint8_t address;

	START: switch (_TWSR) {

	/*****************************
	 * Master write to slave cases
	 *****************************/
	case TWI_SLA_REQ_W_ACK_RTD:
		// Master requested write to slave, ACK has been responded so far
		iwalker = 0;
		// fall through

	case TWI_SLA_DATA_RCV_ACK_RTD:
		// Data byte received from master, ACK has been responded so far
		// save byte
		incomingBuffer[iwalker++] = _TWDR;
		_TWEA = 1;
		// continue reading in data
		receiveByte();
		goto START;
		break;

	case TWI_SLA_DATA_RCV_NACK_RTD:
		// Data byte received from master, ACK has been responded so far
		_TWSR = I2C_IDLE;
		_TWEA = 1;
		break;

		/*****************************
		 * Master read from slave cases
		 *****************************/
	case TWI_SLA_REQ_R_ACK_RTD:
		// Master requested read from slave, ACK has been responded so far
#ifdef ORIG
		owalker = 0;
#endif
		// fall through

	case TWI_SLA_DATA_SND_ACK_RCV:
		// First data byte to send OR Data byte was sent to master, ACK has been received.
		address = incomingBuffer[1];
		_TWDR = outgoingBuffer[address];
		_TWEA = 1;
		// continue sending data
		sendByte();
		goto START;
		break;

	case TWI_SLA_DATA_SND_NACK_RCV:
		// Data byte was sent to master, NACK has been received.
		_TWEA = 1;
		_TWSR = I2C_IDLE;
		break;

		/*****************************
		 * Misc. cases
		 *****************************/
	case TWI_SLA_REPEAT_START:
		// START came in during transmission
		checkStartCondition();
		goto START;

	case TWI_SLA_STOP:
		// Met stop or repeat start, return to idle state
		_TWEA = 1;
		_TWSR = I2C_IDLE;
		break;

	case I2C_IDLE:
		// Idle
	default:
		// Bus error and everything else
		_TWEA = 1;
		break;
	}
}

#endif
