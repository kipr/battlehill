#include <battlecreek/analog_states.hpp>
#include <battlecreek/battery_state.hpp>
#include <battlecreek/digital_states.hpp>
#include <battlecreek/motor_states.hpp>
#include <battlecreek/robot_states.hpp>
#include <battlecreek/servo_states.hpp>

#include <daylite/node.hpp>
#include <daylite/spinner.hpp>

#include "battery.hpp"
#include "led.hpp"

#include <iostream>

using namespace battlecreek;
using namespace daylite;
using namespace std;



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



int main(int argc, char *argv[])
{
  auto n = node::create_node("battlehill");
  
  if(!n->start("127.0.0.1", 8374))
  {
    cerr << "Failed to contact daylite master" << endl;
    return 1;
  }
  
  auto robot_states_pub = n->advertise("robot/robot_states");

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
    battlecreek::robot_states robot_states;
    unsigned short batt_adc = battery_raw_reading();
    robot_states.battery_state.raw_adc = batt_adc;
    robot_states.battery_state.capacity = battery_power_level(batt_adc);
    robot_states_pub->publish(robot_states.bind());



    spinner::spin_once();
  }
  
  return 0;
}
