/*
 * battery.hpp
 *
 *  Created on: Dec 30, 2015
 *      Author: Joshua Southerland
 */

#ifndef SRC_BATTERY_HPP_
#define SRC_BATTERY_HPP_




unsigned short battery_raw_reading();

// TODO: move to battery source/header
// TODO: confusing, this should be % remaining, but currently returns a voltage approximation
float battery_power_level(unsigned short raw_batt = battery_raw_reading());

#endif /* SRC_BATTERY_HPP_ */
