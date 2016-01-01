/*
 * accel.hpp
 *
 *  Created on: Dec 31, 2015
 *      Author: Joshua Southerland
 */

#ifndef SRC_ACCEL_HPP_
#define SRC_ACCEL_HPP_

short accel_x(unsigned char * alt_read_buffer = nullptr);

short accel_y(unsigned char * alt_read_buffer = nullptr);

short accel_z(unsigned char * alt_read_buffer = nullptr);

bool accel_calibrate();

bool accel_calibrated(unsigned char * alt_read_buffer = nullptr);

#endif /* SRC_ACCEL_HPP_ */
