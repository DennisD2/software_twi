/*
 * config.h
 * Silly config file. Check what you need in your application and ignore the rest.
 
 *  Created on: 04.03.2015
 *      Author: dennis
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/** define to enable MOSFET controlling load */
//#define LOAD_CONTROL
/** define to use USI TWI Slave implementation (hardware of attiny ) */
//#define USE_USI_TWI 1
/** define to use software TWI */
#define USE_I2C_SW_DD

/** address of accu module to connect to */
#define SLAVE_ADDRESS 7

#define LED_RED_PIN PB4
#define LED_GREEN_PIN PB3
#define POWER_SWITCH PA4
#define OVERLOAD_MOSFET PA5

#define BUTTON_PIN PA1
#define BUTTON_IPIN PINA1

/* LED pin */
#define DEV_LED_RED_PIN PB4
#define DEV_LED_PORT PORTB
#define DEV_LED_DDR DDRB

/** defines for software I2C */
/** Pin SDA */
#define DEV_SDA_OPIN PA0
#define DEV_SDA_IPIN PINA0
#define DEV_SDA_OPORT PORTA
#define DEV_SDA_IPORT PINA
#define DEV_SDA_DDR DDRA

/** Pin SCL */
#define DEV_SCL_OPIN PA2
#define DEV_SCL_IPIN PINA2
#define DEV_SCL_OPORT PORTA
#define DEV_SCL_IPORT PINA
#define DEV_SCL_DDR DDRA

#endif /* CONFIG_H_ */
