#include <battlecreek/analog_states.hpp>
#include <battlecreek/battery_state.hpp>
#include <battlecreek/digital_states.hpp>
#include <battlecreek/motor_states.hpp>
#include <battlecreek/robot_states.hpp>
#include <battlecreek/servo_states.hpp>
#include <battlecreek/set_digital_state.hpp>
#include <battlecreek/set_motor_state.hpp>
#include <battlecreek/set_servo_state.hpp>


#include <daylite/bson.hpp>
#include <daylite/node.hpp>
#include <daylite/spinner.hpp>

#include "accel.hpp"
#include "analog.hpp"
#include "battery.hpp"
#include "digital.hpp"
#include "gyro.hpp"
#include "led.hpp"
#include "magneto.hpp"
#include "motor.hpp"
#include "servo.hpp"
#include "wallaby_p.hpp"

#include <iostream>

using namespace battlecreek;
using namespace daylite;
using namespace std;


static const unsigned int NUM_ADC = 6; // TODO: move
static const unsigned int NUM_DIG = 16; // TODO: move
static const unsigned int NUM_SERVOS = 4; // TODO: move
static const unsigned int NUM_MOTORS = 4; //TODO: move

namespace
{

  template<typename T>
  inline bson_bind::option<T> safe_unbind(const daylite::bson &raw_msg)
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
  
  void set_motor_state_cb(const daylite::bson & raw_msg, void *)
  {
    const auto msg_option = safe_unbind<set_motor_state>(raw_msg);
    if(msg_option.none()) return;
    
    auto msg = msg_option.unwrap();

    unsigned char port = msg.port;

    // TODO: better range checking and feedback
    if (port >= NUM_MOTORS) return;


    if (msg.position_goal.some()) set_motor_goal_position(port, msg.position_goal.unwrap());
    if (msg.velocity_goal.some()) set_motor_goal_velocity(port, msg.velocity_goal.unwrap());

    if (msg.power.some()) set_motor_pwm(port, msg.power.unwrap());

    if (msg.reset_position.some() && msg.reset_position.unwrap()) clear_motor_bemf(port);

    if (msg.stop.some()) set_motor_stop(port, msg.stop.unwrap());

    if (msg.mode.some()) set_motor_mode(port, msg.mode.unwrap());

    // TODO: should we handle other logic differently if stop is set?, should this happen first or last?
    if (msg.stop.some()) set_motor_stop(port, msg.stop.unwrap());

  }
  
  void set_servo_state_cb(const daylite::bson & raw_msg, void *)
  {
    const auto msg_option = safe_unbind<set_servo_state>(raw_msg);
    if(msg_option.none()) return;

    auto msg = msg_option.unwrap();

    unsigned char port = msg.port;

    // TODO: better range checking and feedback
    if (port >= NUM_SERVOS) return;

    if (msg.position.some()) set_servo_position(port, msg.position.unwrap());

    if (msg.enabled.some()) set_servo_enabled(port, msg.enabled.unwrap());
  }
  
  void set_digital_state_cb(const daylite::bson & raw_msg, void *)
  {
    const auto msg_option = safe_unbind<set_digital_state>(raw_msg);
    if(msg_option.none()) return;
    
    auto msg = msg_option.unwrap();

    unsigned char port = msg.port;

    // TODO: better range checking and feedback
    if (port >= NUM_DIG) return;

    if (msg.output.some()) set_digital_direction(port, msg.output.unwrap());

    if (msg.value.some()) set_digital_value(port, msg.value.unwrap());
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
  robot_states_pub->set_firehose(true);

  auto set_digital_state_sub = n->subscribe("robot/set_digital_state", &set_digital_state_cb);
  auto set_motor_states_sub = n->subscribe("robot/set_motor_state", &set_motor_state_cb);
  auto set_servo_states_sub = n->subscribe("robot/set_servo_state", &set_servo_state_cb);
  
  // TODO: remove digital pin config
  config_led();


  auto wallaby = Private::Wallaby::instance();
  const unsigned int read_buffer_size = wallaby->getBufferSize();

  unsigned char * alt_read_buffer = new unsigned char[read_buffer_size];

  battlecreek::robot_states robot_states;
  robot_states.analog_states.value.resize(NUM_ADC);
  robot_states.digital_states.value.resize(NUM_DIG);
  robot_states.digital_states.output.resize(NUM_DIG);
  robot_states.servo_states.enabled.resize(NUM_SERVOS);
  robot_states.servo_states.position.resize(NUM_SERVOS);
  robot_states.motor_states.motor_state.resize(NUM_MOTORS);

  unsigned long int robot_states_pub_count = 0;

  for(;;)
  {

    // get all robot state data from the co-processor
    wallaby->readToAltBuffer(alt_read_buffer, read_buffer_size);

    // update all of the robot state data

    // accelerometer
    robot_states.imu_state.accel_state.x = accel_x(alt_read_buffer);
    robot_states.imu_state.accel_state.y = accel_y(alt_read_buffer);
    robot_states.imu_state.accel_state.z = accel_z(alt_read_buffer);
    robot_states.imu_state.accel_state.calibrated = accel_calibrated(alt_read_buffer);

    // analog
    for (unsigned int i = 0; i < NUM_ADC; ++i) robot_states.analog_states.value[i] = analog_value(i, alt_read_buffer);

    // battery
    unsigned short batt_adc = battery_raw_reading(alt_read_buffer);
    robot_states.battery_state.raw_adc = batt_adc;
    robot_states.battery_state.capacity = battery_power_level(batt_adc);

    // buttons

    // digitals
    for (unsigned int i = 0; i < NUM_DIG; ++i)
    {
      robot_states.digital_states.value[i] = digital_value(i, alt_read_buffer);
      robot_states.digital_states.output[i] = digital_output(i, alt_read_buffer);
    }

    // gyro
    robot_states.imu_state.gyro_state.x = gyro_x(alt_read_buffer);
    robot_states.imu_state.gyro_state.y = gyro_y(alt_read_buffer);
    robot_states.imu_state.gyro_state.z = gyro_z(alt_read_buffer);
    robot_states.imu_state.gyro_state.calibrated = gyro_calibrated(alt_read_buffer);

    // magnetometer
    robot_states.imu_state.magneto_state.x = magneto_x(alt_read_buffer);
    robot_states.imu_state.magneto_state.y = magneto_y(alt_read_buffer);
    robot_states.imu_state.magneto_state.z = magneto_z(alt_read_buffer);
    robot_states.imu_state.magneto_state.calibrated = magneto_calibrated(alt_read_buffer);

    // motors
    for (unsigned int i = 0; i < NUM_MOTORS; ++i)
    {
      robot_states.motor_states.motor_state[i].direction = get_motor_direction(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].done = get_motor_done(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].goal_position = get_motor_goal_position(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].goal_velocity = get_motor_goal_velocity(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].mode = get_motor_mode(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].pid_active = get_motor_pid_active(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].position = get_motor_bemf(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].power = get_motor_pwm(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].stop = get_motor_stop(i, alt_read_buffer);
      robot_states.motor_states.motor_state[i].velocity = get_motor_bemf_vel(i, alt_read_buffer);
    }

    // servos
    for (unsigned int i = 0; i < NUM_SERVOS; ++i)
    {
      robot_states.servo_states.enabled[i] = get_servo_enabled(i, alt_read_buffer);
      robot_states.servo_states.position[i] = get_servo_position(i, alt_read_buffer);
    }

    // publish robot state data
    robot_states_pub_count += 1;
    robot_states.seq = robot_states_pub_count;
    robot_states.update_count = Private::Wallaby::instance()->getUpdateCount();
    robot_states_pub->publish(robot_states.bind());

    // check for new messages
    spinner::spin_once();
  }
  
  return 0;
}
