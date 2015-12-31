#include <battlecreek/analog_states.hpp>
#include <battlecreek/battery_state.hpp>
#include <battlecreek/digital_states.hpp>
#include <battlecreek/motor_states.hpp>
#include <battlecreek/robot_states.hpp>
#include <battlecreek/servo_states.hpp>

#include <daylite/node.hpp>
#include <daylite/spinner.hpp>

#include "accel.hpp"
#include "analog.hpp"
#include "battery.hpp"
#include "gyro.hpp"
#include "led.hpp"
#include "magneto.hpp"
#include "wallaby_p.hpp"

#include <iostream>

using namespace battlecreek;
using namespace daylite;
using namespace std;


static const unsigned int NUM_ADC = 6; // TODO: move

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

  auto set_digital_states_sub = n->subscribe("robot/set_digital_states", &set_digital_states_cb);
  auto set_motor_states_sub = n->subscribe("robot/set_motor_states", &set_motor_states_cb);
  auto set_servo_states_sub = n->subscribe("robot/set_servo_states", &set_servo_states_cb);
  
  // TODO: remove digital pin config
  config_led();


  auto wallaby = Private::Wallaby::instance();
  const unsigned int read_buffer_size = wallaby->getBufferSize();

  unsigned char * alt_read_buffer = new unsigned char[read_buffer_size];

  battlecreek::robot_states robot_states;
  robot_states.analog_states.value.resize(NUM_ADC);

  for(;;)
  {

    // get all robot state data from the co-processor
    wallaby->readToAltBuffer(alt_read_buffer, read_buffer_size);

    // update all of the robot state data

    // accelerometer
    robot_states.imu_state.accel_state.x = accel_x();
    robot_states.imu_state.accel_state.y = accel_y();
    robot_states.imu_state.accel_state.z = accel_z();
    robot_states.imu_state.accel_state.calibrated = accel_calibrated();

    // analog
    for (unsigned int i = 0; i < NUM_ADC; ++i) robot_states.analog_states.value[i] = analog_value(i, alt_read_buffer);

    // battery
    unsigned short batt_adc = battery_raw_reading(alt_read_buffer);
    robot_states.battery_state.raw_adc = batt_adc;
    robot_states.battery_state.capacity = battery_power_level(batt_adc);

    // buttons

    // digitals

    // gyro
    robot_states.imu_state.gyro_state.x = gyro_x();
    robot_states.imu_state.gyro_state.y = gyro_y();
    robot_states.imu_state.gyro_state.z = gyro_z();
    robot_states.imu_state.gyro_state.calibrated = gyro_calibrated();

    // magnetometer
    robot_states.imu_state.magneto_state.x = magneto_x();
    robot_states.imu_state.magneto_state.y = magneto_y();
    robot_states.imu_state.magneto_state.z = magneto_z();
    robot_states.imu_state.magneto_state.calibrated = magneto_calibrated();

    // motors

    // servos

    // publish robot state data
    robot_states_pub->publish(robot_states.bind());

    // check for new messages
    spinner::spin_once();
  }
  
  return 0;
}
