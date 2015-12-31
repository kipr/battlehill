/*
 * gyro.cpp
 *
 *  Created on: Dec 31, 2015
 *      Author: Joshua Southerland
 */

#include "gyro.hpp"
#include "wallaby_p.hpp"
#include "wallaby_regs_p.hpp"


short gyro_x()
{
    return static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_GYRO_X_H));
}

short gyro_y()
{
  signed short val = static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_GYRO_Y_H));
}

short gyro_z()
{
  signed short val = static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_GYRO_Z_H));
}

bool gyro_calibrate()
{
  // TODO
  return true;
}

bool gyro_calibrated()
{
  // TODO
  return true;
}

