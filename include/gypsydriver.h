/*
 * gypsydriver.h
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#ifndef GYPSY_DRIVER_H_
#define GYPSY_DRIVER_H_

// Constant Definitions
#define HBRIDGE_ADDRESS     	128
#define HBRIDGE_BAUD_RATE   	9600

#define HBRIDGE_MIN_VOLTAGE 	34  // Cutoff voltage of roughly 10V
#define HBRIDGE_DEADBAND    	3   // Deadband cutoff of 3 counts
#define HBRIDGE_TIMEOUT			200 // Serial timeout of 200ms

#define HBRIDGE_CMD_MIN    		-127
#define HBRIDGE_CMD_MAX     	127

#define HBRIDGE_SERIAL_TX_PIN	P8_37
#define HBRIDGE_SHUTDOWN		P8_41

// Constant Definitions
#define SENSOR_I2C				P9_20
#define LEFT_SENSOR_ADR			0x60
#define RIGHT_SENSOR_ADR		0x61

// Motion Constants
#define SPEED_MIN          		-100
#define SPEED_MAX           	100

#define DIR_MIN            		-100
#define DIR_MAX             	100

#endif /* GYPSYDRIVER_H_ */
