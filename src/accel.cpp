/*
 * accel.cpp
 *
 *  Created on: Dec 31, 2015
 *      Author: Joshua Southerland
 */

#include "accel.hpp"
#include "wallaby_p.hpp"
#include "wallaby_regs_p.hpp"


short accel_x()
{
    return static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_ACCEL_X_H));
}

short accel_y()
{
  signed short val = static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_ACCEL_Y_H));
}

short accel_z()
{
  signed short val = static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_ACCEL_Z_H));
}

bool accel_calibrate()
{
  // TODO
  return true;
}

bool accel_calibrated()
{
  // TODO
  return true;
}
