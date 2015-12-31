/*
 * magneto.cpp
 *
 *  Created on: Dec 31, 2015
 *      Author: Joshua Southerland
 */

#include "magneto.hpp"

#include "accel.hpp"
#include "wallaby_p.hpp"
#include "wallaby_regs_p.hpp"


short magneto_x()
{
    return static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_MAG_X_H));
}

short magneto_y()
{
  signed short val = static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_MAG_Y_H));
}

short magneto_z()
{
  signed short val = static_cast<signed short>(Private::Wallaby::instance()->readRegister16b(REG_RW_MAG_Z_H));
}

bool magneto_calibrate()
{
  // TODO
  return true;
}

bool magneto_calibrated()
{
  // TODO
  return true;
}
