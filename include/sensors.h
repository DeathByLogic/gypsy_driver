/*
 * sensors.h
 *
 *  Created on: Mar 27, 2016
 *      Author: daniel
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "beagleIO.h"

// Constant Definitions
#define PI					(float)3.14159

#define WHEEL_BASE			13.25		// Distance between wheels
#define WHEEL_DIM			4.875		// Diameter of wheels
#define ENC_COUNT			100			// Number of encoder counts per revolution

#define PERIOD_DIVIDER		1E6			// Microseconds per second

// Type Definitions
typedef struct {
	float		x;
	float		y;
} Point;

typedef struct {
	Point		location;
	float		heading;
} Posistion;

// Function Definitions
void read_sensor(beagleI2C *, const char, const char, const size_t, void *);
void update_location(Posistion *, long, long);

#endif /* SENSORS_H_ */
