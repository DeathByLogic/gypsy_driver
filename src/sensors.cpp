/*
 * sensors.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: daniel
 */

#include <math.h>

#include "beagleIO.h"
#include "sensors.h"

using namespace std;

void read_sensor(beagleI2C *port, const char i2cAddress, const char startAddress, const size_t size, void *memAdr) {
  char tempArray[size];

  // Write starting location to device
  port->writeTo(&startAddress, i2cAddress, sizeof(startAddress));

  // Read indicated number of bytes
  port->requestFrom(tempArray, i2cAddress, size, true);

  // Copy data into memory
  memcpy(memAdr, &tempArray, size);
}

// Update the calculated position with new data
void update_location(Posistion *posistion, long left_encoder_count, long right_encoder_count) {
	float theta_l;
	float theta_r;
	float theta_sum;

	float delta_x = 0.0;
	float delta_y = 0.0;

	// Left Wheel
	theta_l = -((float)left_encoder_count * WHEEL_DIM * PI / ENC_COUNT) / WHEEL_BASE;

	delta_x += 0.5 * WHEEL_BASE * (sin(posistion->heading) - sin(posistion->heading + theta_l));
	delta_y += 0.5 * WHEEL_BASE * (cos(posistion->heading + theta_l) - cos(posistion->heading));

	// Right Wheel
	theta_r = ((float)right_encoder_count * WHEEL_DIM * PI / ENC_COUNT) / WHEEL_BASE;

	delta_x += 0.5 * WHEEL_BASE * (sin(posistion->heading + theta_r) - sin(posistion->heading));
	delta_y += 0.5 * WHEEL_BASE * (cos(posistion->heading) - cos(posistion->heading + theta_r));

	// Sum up current theta and delta
	theta_sum = posistion->heading + (theta_r + theta_l);

	// Keep theta within +- PI
	if (theta_sum > PI) {
		theta_sum -= 2 * PI;
	}

	if (theta_sum < -PI) {
		theta_sum += 2 * PI;
	}

	// Update external variables
	posistion->location.x += delta_x;
	posistion->location.y += delta_y;
	posistion->heading = theta_sum;
}
