#include <battlecreek/analog_states.hpp>
#include <battlecreek/battery_state.hpp>
#include <battlecreek/digital_states.hpp>
#include <battlecreek/motor_states.hpp>
#include <battlecreek/servo_states.hpp>

#include <daylite/node.hpp>
#include <daylite/spinner.hpp>

#include "wallaby_regs_p.hpp"
#include "wallaby_p.hpp"

#include <iostream>

using namespace battlecreek;
using namespace daylite;
using namespace std;


static const unsigned int LED_PIN_NUM = 12;

namespace
{

  template<typename T>
  inline bson_bind::option<T> safe_unbind(const bson_t *raw_msg)
  {
    using namespace bson_bind;
    T ret;
    try
    {
      ret = T::unbind(raw_msg);
    }
    catch(const invalid_argument &e)
    {
      cerr << e.what() << endl;
      return none<T>();
    }
    
    return some(ret);
  }
  
  void set_motor_states_cb(const bson_t *raw_msg, void *)
  {
    const auto msg_option = safe_unbind<motor_states>(raw_msg);
    if(msg_option.none()) return;
    
    auto msg = msg_option.unwrap();
  }
  
  void set_servo_states_cb(const bson_t *raw_msg, void *)
  {
    const auto msg_option = safe_unbind<servo_states>(raw_msg);
    if(msg_option.none()) return;
    
    auto msg = msg_option.unwrap();
  }
  
  void set_analog_states_cb(const bson_t *raw_msg, void *)
  {
    const auto msg_option = safe_unbind<analog_states>(raw_msg);
    if(msg_option.none()) return;
    
    auto msg = msg_option.unwrap();
  }
  
  void set_digital_states_cb(const bson_t *raw_msg, void *)
  {
    const auto msg_option = safe_unbind<digital_states>(raw_msg);
    if(msg_option.none()) return;
    
    auto msg = msg_option.unwrap();
  }
}

void config_led()
{
  unsigned short outputs = Private::Wallaby::instance()->readRegister16b(REG_RW_DIG_OE_H);

  //  bit = 1 for output, 0 for input
  outputs |= (1 << LED_PIN_NUM);

  Private::Wallaby::instance()->writeRegister16b(REG_RW_DIG_OE_H, outputs);

  usleep(1000);
}

// TODO: move/remove
void blink_led()
{

  // led (on?)
  unsigned short out;
  out = Private::Wallaby::instance()->readRegister16b(REG_RW_DIG_OUT_H);
  out |= (1 << LED_PIN_NUM);
  Private::Wallaby::instance()->writeRegister16b(REG_RW_DIG_OUT_H, out);

  usleep(20000);

  // led (off?)
  out = Private::Wallaby::instance()->readRegister16b(REG_RW_DIG_OUT_H);
  out &= ~(1 << LED_PIN_NUM);
  Private::Wallaby::instance()->writeRegister16b(REG_RW_DIG_OUT_H, out);
  usleep(20000);
}


// TODO: move to battery source/header
float power_level()
{
  // piece the 12-bit ADC result back together
  unsigned short raw_batt_adc = Private::Wallaby::instance()->readRegister16b(REG_RW_BATT_H);

  // calculate voltage based on linear curve-fitting
  float batt_voltage = -0.02070635f + 0.009071161f * static_cast<float>(raw_batt_adc);

  // FIXME   convert ADC->capacity  or  voltage->capacity

  return batt_voltage;
}


int main(int argc, char *argv[])
{
  auto n = node::create_node("battlehill");
  
  if(!n->start("127.0.0.1", 8374))
  {
    cerr << "Failed to contact daylite master" << endl;
    return 1;
  }
  
  auto analog_states_pub = n->advertise("robot/analog_states");
  auto battery_state_pub = n->advertise("robot/battery_state");
  auto digital_states_pub = n->advertise("robot/digital_states");
  auto motor_states_pub = n->advertise("robot/motor_states");
  auto servo_states_pub = n->advertise("robot/servo_states");

  auto set_analog_states_sub = n->subscribe("robot/set_analog_states", &set_analog_states_cb);
  auto set_digital_states_sub = n->subscribe("robot/set_digital_states", &set_digital_states_cb);
  auto set_motor_states_sub = n->subscribe("robot/set_motor_states", &set_motor_states_cb);
  auto set_servo_states_sub = n->subscribe("robot/set_servo_states", &set_servo_states_cb);
  
  // TODO: remove digital pin config
  config_led();

  for(;;)
  {
    blink_led();

    // publish battery voltage
    battlecreek::battery_state battery_state;
    battery_state.capacity = power_level();
    std::cout << "batt voltage = " << std::to_string(battery_state.capacity) << std::endl;
    battery_state_pub->publish(battery_state.bind());

    spinner::spin_once();
  }
  
  return 0;
}
