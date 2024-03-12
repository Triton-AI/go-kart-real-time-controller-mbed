/**
 * @file actuation_controller.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-02
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "actuation_controller.hpp"
#include "Kernel.h"
#include "PinNamesTypes.h"
#include "PwmIn.h"
#include "ThisThread.h"
#include "Thread.h"
#include "Tools/global_profilers.hpp"
#include "can_helper.h"
#include "config.hpp"
#include "vesc_can_helper.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <map>
#include <string>
#include <utility>

/**
 *
 * @brief VESC CAN message ID generator functions
 * These functions generate the unique CAN message IDs for sending commands and
 * requesting information to/from VESC motor controllers. They are used in
 * conjunction with the CAN protocol to communicate with the VESC motor
 * controllers.
 * @note For more information on communicating with VESC controllers with CAN,
 * see https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can.
 * @param vesc_id The unique identifier of the VESC motor controller to
 * communicate with
 * @return The unique CAN message ID for the specified VESC motor controller and
 * message type
 */

#define VESC_RPM_EXTENDED_ID 0x03
#define VESC_CURRENT_EXTENDED_ID 1
#define VESC_POSITION_EXTENDED_ID 4
#define VESC_STATUS_EXTENDED_ID_PACKET_1 0x09
#define VESC_STATUS_EXTENDED_ID_PACKET_4 0x10

/**
 *
 * @brief Generate unique CAN message ID for RPM command
 * This function generates the unique CAN message ID for sending RPM commands to
 * a VESC motor controller with the specified identifier.
 * @param vesc_id The unique identifier of the VESC motor controller to send the
 * RPM command to
 * @return The unique CAN message ID for sending the RPM command to the
 * specified VESC motor controller
 */
static constexpr uint32_t VESC_RPM_ID(const uint32_t &vesc_id) {
  // The RPM command has an extended ID of 0x03, and the VESC ID is concatenated
  // to form the unique message ID.
  return (static_cast<uint32_t>(VESC_RPM_EXTENDED_ID) << sizeof(uint8_t) * 8) |
         static_cast<uint32_t>(vesc_id);
}

/**
 *
 * @brief Generate unique CAN message ID for current command
 * This function generates the unique CAN message ID for sending current
 * commands to a VESC motor controller with the specified identifier.
 * @param vesc_id The unique identifier of the VESC motor controller to send the
 * current command to
 * @return The unique CAN message ID for sending the current command to the
 * specified VESC motor controller
 */
static constexpr uint32_t VESC_CURRENT_ID(const uint32_t &vesc_id) {
  return (static_cast<uint32_t>(VESC_CURRENT_EXTENDED_ID)
          << sizeof(uint8_t) * 8) |
         static_cast<uint32_t>(vesc_id);
}

/**
 *
 * @brief Generate unique CAN message ID for position control command
 * This function generates the unique CAN message ID for sending position
 * control commands to a VESC motor controller with the specified identifier.
 * @param vesc_id The unique identifier of the VESC motor controller to send the
 * current command to
 * @return The unique CAN message ID for sending the current command to the
 * specified VESC motor controller
 */
static constexpr uint32_t VESC_POSITION_ID(const uint32_t &vesc_id) {
  return (static_cast<uint32_t>(VESC_POSITION_EXTENDED_ID)
          << sizeof(uint8_t) * 8) |
         static_cast<uint32_t>(vesc_id);
}

/**
 *
 * @brief Generate unique CAN message ID for status request
 * This function generates the unique CAN message ID for incoming status
 * information from a VESC motor controller with the specified identifier. To
 * see which specific ones, please check
 * https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md#status-commands
 * @param vesc_id The unique identifier of the VESC motor controller to get
 * status information from
 * @return The unique CAN message ID for incoming status information from the
 * specified VESC motor controller
 */
static constexpr uint32_t VESC_STATUS_ID_PACKET_1(const uint32_t &vesc_id) {
  return (static_cast<uint32_t>(VESC_STATUS_EXTENDED_ID_PACKET_1)
          << sizeof(uint8_t) * 8) |
         static_cast<uint32_t>(vesc_id);
}

/**
 *
 * @brief Generate unique CAN message ID for status request
 * This function generates the unique CAN message ID for incoming status
 * information from a VESC motor controller with the specified identifier. To
 * see which specific ones, please check
 * https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md#status-commands
 * @param vesc_id The unique identifier of the VESC motor controller to get
 * status information from
 * @return The unique CAN message ID for incoming status information from the
 * specified VESC motor controller
 */
static constexpr uint32_t VESC_STATUS_ID_PACKET_4(const uint32_t &vesc_id) {
  return (static_cast<uint32_t>(VESC_STATUS_EXTENDED_ID_PACKET_4)
          << sizeof(uint8_t) * 8) |
         static_cast<uint32_t>(vesc_id);
}

namespace tritonai {
namespace gkc {

/**
 *
 * @brief Clamp a value between a minimum and maximum range
 * This function takes a value and clamps it to a specified range between a
 * minimum and maximum value.
 * @tparam T The type of the value to clamp (must be comparable with the <
 * operator and support assignment)
 * @param val The value to clamp
 * @param min The minimum value of the range to clamp to
 * @param max The maximum value of the range to clamp to
 * @return The value clamped to the specified range between min and max
 */
template <typename T>
constexpr T clamp(const T &val, const T &min, const T &max) {
  if (val > max) {
    return max;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}

// Deprecated function. It was uesed in the past with an old motror controller
// that came with the kart. It should not be used. Please test it code compieles
// deleting it and remove it. I dont want to remove any code as I can't test it.
template <typename T> constexpr T non_linear_map(const T &val) {
  return (exp(val * val * val) - 1) / (exp(1) - 1);
}

/**
 *
 * @brief Map a value from one range to another range
 * This function takes a value from a source range and maps it to a destination
 * range. The input value is first clamped to the source range between
 * source_min and source_max. Then, it is linearly interpolated to a value
 * between 0 and 1, where 0 represents the minimum of the source range and 1
 * represents the maximum of the source range. This normalized value is then
 * linearly interpolated to a value between dest_min and dest_max, which
 * represents the corresponding value in the destination range. The resulting
 * value is then returned.
 * @tparam S The type of the input source value to map (must be comparable with
 * the < operator and support assignment)
 * @tparam D The type of the mapped output destination value (must support
 * assignment)
 * @param source The input value to map to the destination range
 * @param source_min The minimum value of the source range
 * @param source_max The maximum value of the source range
 * @param dest_min The minimum value of the destination range
 * @param dest_max The maximum value of the destination range
 * @return The input value mapped to the destination range
 */
template <typename S, typename D>
constexpr D map_range(const S &source, const S &source_min, const S &source_max,
                      const D &dest_min, const D &dest_max) {
  // Clamp the input value to the source range between source_min and source_max
  float source_f = clamp<S>(source, source_min, source_max);

  // Normalize the clamped value between 0 and 1 based on its position within
  // the source range
  source_f = (source_f - source_min) / (source_max - source_min);

  // Map the normalized value to the destination range
  return static_cast<D>(source_f * (dest_max - dest_min) + dest_min);
}

/**
 *
 * @brief Convert an angle in degrees to radians
 * This function takes an angle in degrees and converts it to radians. The input
 * angle is multiplied by the constant M_PI/180.0 to obtain its equivalent in
 * radians, and then cast to the output type.
 * @tparam S The type of the input angle in degrees (must support multiplication
 * with a float)
 * @tparam D The type of the output angle in radians (must support assignment)
 * @param deg The input angle in degrees to convert to radians
 * @return The input angle converted to radians
 */
template <typename S, typename D> constexpr D deg_to_rad(const S &deg) {
  return static_cast<D>(deg * M_PI / 180.0);
}

/**
 *
 * @brief Convert an angle in radians to degrees
 * This function takes an angle in radians and converts it to degrees. The input
 * angle is multiplied by the constant 180.0/M_PI to obtain its equivalent in
 * degrees, and then cast to the output type.
 * @tparam S The type of the input angle in radians (must support multiplication
 * with a float)
 * @tparam D The type of the output angle in degrees (must support assignment)
 * @param rad The input angle in radians to convert to degrees
 * @return The input angle converted to degrees
 */
template <typename S, typename D> constexpr D rad_to_deg(const S &rad) {
  return static_cast<D>(rad * 180.0 / M_PI);
}

/**
 *
 * @brief Map steering angle to motor angle
 * This function maps the steering angle in radians to the corresponding motor
 * angle in radians, based on a predefined lookup table. The lookup table is a
 * map between motor angles and steering angles, obtained through empirical
 * measurements of the relationship between the two angles. The function finds
 * the closest two entries in the map that bound the input steering angle and
 * linearly interpolates the motor angle between these two entries using the
 * map_range() function. The resulting motor angle is then returned.
 *
 * This look up table is defined in config.hpp as STERING_MAPPTING
 *
 * @param steer_angle The steering angle in radians to map to a motor angle
 * @return The corresponding motor angle in radians
 */
float map_steer2motor(float steer_angle) {
  // Define the lookup table as a map between motor angles and steering angles
  std::map<float, float> mapping = STERING_MAPPTING;

  // Save sign
  int sign = steer_angle >= 0 ? 1 : -1;

  // Find the two entries in the map that bound the input steering angle and
  // linearly interpolate between them
  for (auto it = mapping.begin(); it != mapping.end(); it++) {
    if ((std::next(it))->second >= sign * steer_angle) {

      auto returned =
          sign * map_range<float, float>(sign * steer_angle, it->second,
                                         std::next(it)->second, it->first,
                                         std::next(it)->first);

      return returned;
    }
  }

  // If the input steering angle is out of bounds of the look-up table, return 0
  return 0;
}

/**
 *
 * @brief Map motor angle to steering angle
 * This function maps the motor angle in radians to the corresponding steering
 * angle in radians, based on a predefined lookup table. The lookup table is a
 * map between motor angles and steering angles, obtained through empirical
 * measurements of the relationship between the two angles. The function finds
 * the closest two entries in the map that bound the input motor angle and
 * linearly interpolates the steering angle between these two entries using the
 * map_range() function. The resulting steering angle is then returned.
 *
 * This look up table is defined in config.hpp as STERING_MAPPTING
 *
 * @param motor_angle The motor angle in radians to map to a steering angle
 * @return The corresponding steering angle in radians
 */
float map_motor2steer(float motor_angle) {
  std::map<float, float> mapping = STERING_MAPPTING;

  // Subtract pi (180) from the input motor angle. Actually it is because the
  // neutral angle 0 is sometimes represented as 180 in the system. It is a bit
  // of a mess. It is done because an encoder that was used was mounted so it
  // had the point of overflow (where it goes from 0 to 360, or from 360 to 0)
  // this point is in the opposite angle to nwutral, so neutras is actually
  // represented as 180. What makes it more a mess is that this overflow point
  // is also configurable by software. I'll try to explain all this better at
  // some point.
  motor_angle -= M_PI;

  // Determine the sign
  int sign = motor_angle >= 0 ? 1 : -1;

  // Find the two entries in the map that bound the input motor angle and
  // linearly interpolate
  for (auto it = mapping.begin(); it != mapping.end(); it++)
    if ((std::next(it))->first >= sign * motor_angle)
      return sign * map_range<float, float>(sign * motor_angle, it->first,
                                            std::next(it)->first, it->second,
                                            std::next(it)->second);

  // If the input motor angle is out of bounds of the look-up table, return 0
  return 0;
}

/**
 * @brief check if the kart is ready to drive
 *
 * So far the kart doesn't do any self checks before beeing ready to drive
 *
 * TODO: It would be nice to have some automatic tests when starting the RTC.
 * The kart would be rady to start untill tese tests are passed
 *
 * @return true if kart is ready to drive
 * @return false if kart is not ready to drive
 */

bool ActuationController::is_ready() { return true; }

/**
 * @brief Fills the sesor packet with the latest sensor data
 *
 * The populate_reading() function is called to populate a SensorGkcPacket with
 * sensor information from the ActuationController. The SensorGkcPacket is used
 * to send sensor data from the autonomous kart to the main computer.
 *
 * The function retrieves the sensor data from the ActuationSensors object,
 * which is stored in the ActuationController. It then populates the
 * corresponding fields in the SensorGkcPacket
 *
 * This function has to be devine as ActuationController inherits from
 * ISensorProvider. ISensorProvider will be calling this function to get the
 * latest sensor data.
 *
 * sensors sctruct is filled with the latest infromation by the function
 * ActuatorCaontroller::sensor_poll_thread_impl() which is running continously
 * on a thread
 *
 * @param pkt GkcPacket that function populates with the latest sensor readings.
 */

void ActuationController::populate_reading(SensorGkcPacket &pkt) {
  pkt.values.steering_angle_rad = sensors.steering_rad;
  pkt.values.brake_pressure = sensors.brake_psi;
  pkt.values.wheel_speed_fl = sensors.fl_rad;
  pkt.values.wheel_speed_fr = sensors.fr_rad;
  pkt.values.wheel_speed_rl = sensors.rl_rad;
  pkt.values.wheel_speed_rr = sensors.rr_rad;
  pkt.values.servo_angle_rad = map_motor2steer(
      sensors.steering_rad); // static_cast<float>(sensors.steering_output);
}

PwmIn steer_encoder(STEER_ENCODER_PIN);
CAN CAN_1(CAN1_RX, CAN1_TX, CAN1_BAUDRATE);
CAN CAN_2(CAN2_RX, CAN2_TX, CAN2_BAUDRATE);
DigitalIn leftLimitSwitch(RIGHT_LSWITCH);
DigitalIn rightLimitSwitch(LEFT_LSWITCH);
Timer timey;

/**
 * @brief Construct a new Actuation Controller:: Actuation Controller object
 *
 * @param logger The Actuation Controller uses a ponter to a logger to print the
 * information, but instead of printing it sends the strings to the MRC which
 * logs it and prints them if it is configured to do so.
 *
 *
 * The ActuationController::ActuationController() function is the constructor of
 * the ActuationController class. It initializes the class members and sets up
 * the threads that will control the actuators and retrieve sensor data.
 *
 * The constructor first calls the Watchable and ISensorProvider constructors to
 * set up the interval for checking if the ActuationController is still
 * operational and to declare the class as a sensor provider.
 *
 * Next, the current_steering_cmd is initialized to the neutral steering angle,
 * which is converted from degrees to radians using the deg_to_rad() function.
 * The steering_pid object is also initialized using the steering_pid_coeff
 * object declared just above, which contains the PID parameters defined in
 * config.hpp.
 *
 * The constructor initializes the logger object that is used for logging.
 *
 * The function then sets up the limit switches by setting their mode to PullUp.
 * This allows the limit switches to detect when the steering reaches the end of
 * its range of motion.
 *
 * The constructor then starts the threads that control the actuators and
 * retrieve sensor data. The throttle_thread, steering_thread,
 * steering_pid_thread, brake_thread, and sensor_poll_thread threads are started
 * by calling their respective callback functions. These functions are defined
 * in the ActuationController class and implement the logic for controlling the
 * actuators and retrieving sensor data.
 *
 */

ActuationController::ActuationController(ILogger *logger)
    : Watchable(DEFAULT_ACTUATION_INTERVAL_MS,
                DEFAULT_ACTUATION_LOST_TOLERANCE_MS,
                "ActuationController"),
      ISensorProvider(),
      current_steering_cmd(deg_to_rad<int32_t, float>(NEUTRAL_STEER_DEG)),
      logger(logger) {

  leftLimitSwitch.mode(PullUp);
  rightLimitSwitch.mode(PullUp);

  CAN_2.reset();
  CAN_2.frequency(CAN2_BAUDRATE);

  can_transmit_thread.start(
      callback(this, &ActuationController::can_transmit_thread_impl));
  throttle_thread.start(
      callback(this, &ActuationController::throttle_thread_impl));
  steering_thread.start(
      callback(this, &ActuationController::steering_thread_impl));

  // brake_thread.start(callback(this,
  // &ActuationController::brake_thread_impl));
  // sensor_poll_thread.start(
  //     callback(this, &ActuationController::sensor_poll_thread_impl));
}

/**
 * @brief This is a function ran on a thread that runs continously to control
 * the throttle actuator of the autonomous kart. It retrieves the desired
 * throttle command from the throttle_cmd_queue and converts it to a command
 * that can be sent to the VESC (motor controller) using the CAN bus.
 *
 * The function first declares a float pointer cmd that will be used to store
 * the retrieved command from the throttle_cmd_queue.
 *
 * Next, the function enters a while loop that runs continuously until the
 * thread is killed. The flags_get() function is used to check if the thread has
 * been signaled to stop.
 *
 * Inside the while loop, the function tries to retrieve a command from the
 * throttle_cmd_queue using the try_get_for() function. This function waits for
 * a specified amount of time for a command to be added to the queue. In this
 * case, the function waits indefinitely until a command is received.
 *
 * Once a command is received, the function sets the current_throttle_cmd member
 * variable to the received command, ensuring that it is within the maximum and
 * minimum allowed values. The function then converts the command to a VESC
 * command by dividing the command by the CONST_ERPM2MS constant. This
 * conversion is required because the VESC motor controller uses ERPM
 * (electrical revolutions per minute) values to control the motor.
 *
 * The function then deletes the cmd pointer and creates a message buffer to
 * store the VESC current command. The buffer_append_int32() function is used to
 * copy the VESC command to the message buffer.
 *
 * Finally, the function sends the message to the VESC using the
 * CAN_THROTTLE.write() function. The message is sent as a CANMessage with the
 * VESC_RPM_ID and THROTTLE_VESC_ID identifiers. CAN_THROTTLE is defined in
 * config.hpp and can be the CAN1 or CAN2 ports.
 *
 * mbed CAN
 * https://os.mbed.com/docs/mbed-os/v6.16/apis/other-driver-apis.html
 *
 * VESC CAN
 * https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can
 *
 */

void ActuationController::throttle_thread_impl() {
  ThisThread::sleep_for(1s);

  float *cmd;
  while (!ThisThread::flags_get()) {
    throttle_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &cmd);
    current_throttle_cmd =
        clamp<float>(*cmd, -MAX_THROTTLE_MS, MAX_THROTTLE_MS);
    const int32_t vesc_current_cmd = current_throttle_cmd / CONST_ERPM2MS;

    uint8_t message[4] = {0, 0, 0, 0};
    int32_t idx = 0;
    buffer_append_int32(&message[0], vesc_current_cmd, &idx);

    CANMessage *cMsg;

    cMsg = new CANMessage(VESC_CURRENT_ID(THROTTLE_VESC_ID), &message[0],
                          sizeof(message), CANData, CANExtended);
    
    if(!can_cmd_queue.try_put(cMsg)){
      delete cMsg;
    }
    
    delete cmd;

    ThisThread::sleep_for(2ms);
  }
}

void ActuationController::can_transmit_thread_impl() {
  ThisThread::sleep_for(1s);

  CANMessage *c_msg;
  while (!ThisThread::flags_get()) {
    can_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &c_msg);

    if (!CAN_2.write(*c_msg)) {
      // std::cout << "Failed to send CAN message\n";
      CAN_2.reset();
      CAN_2.frequency(CAN2_BAUDRATE);
    }

    delete c_msg;

    // ThisThread::sleep_for(10ms);
  }
}

/**
 *
 * @brief Implementation of the steering control thread
 * This function is the implementation of the thread steering_thread. It only
 * sets the current steering command variable(current_steering_cmd) based on the
 * incoming messages from the MRC. The function retrieves the command from the
 * steering command queue, maps it to the appropriate motor angle, and sets the
 * current steering command (current_steering_cmd) to the mapped value. The
 * function also clamps the command to prevent it from exceeding the maximum and
 * minimum allowable steering angles.
 *
 * The steering_pid_thread_impl() is going to be using the current_steering_cmd
 * as the target position for the steering motor and it is going to achieve the
 * possition with a PID controller feedback loop.
 */

void ActuationController::steering_thread_impl() {
  // Wait for 1 second before starting to allow the rest of the system to
  // initialize
  ThisThread::sleep_for(1s);
  float *cmd;
  // float prev_vesc_steering_cmd = 0;
  while (!ThisThread::flags_get()) // Forever until thread is killed
  {
    steering_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &cmd);

    // std::cout << "Steering Angle Before Map Before 1st Clamp (Deg): "
    //           << std::setprecision(2) << *cmd << "\n";

    *cmd = clamp<float>(*cmd, deg_to_rad<float, float>(MIN__WHEEL_STEER_DEG),
                        deg_to_rad<float, float>(MAX__WHEEL_STEER_DEG));

    // std::cout << "Steering Angle Before Map After 1st Clamp (Deg): "
    //           << std::setprecision(2) << rad_to_deg<float, float>(*cmd) <<
    //           "\n";

    *cmd = map_steer2motor(*cmd);

    // std::cout << "Steering Angle After Map Before 2nd Clamp (Deg): "
    //           << std::setprecision(2) << rad_to_deg<float, float>(*cmd) <<
    //           "\n";

    *cmd = clamp<float>(*cmd, deg_to_rad<float, float>(MIN_STEER_DEG),
                        deg_to_rad<float, float>(MAX_STEER_DEG));

    // std::cout << "Steering Angle After Map After 2nd Clamp (Deg): "
    //           << std::setprecision(2) << rad_to_deg<float, float>(*cmd) <<
    //           "\n";

    angle_steering_cmd = *cmd; //  this is in radians
    const float vesc_steering_cmd = rad_to_deg<float, float>(
        fmod(angle_steering_cmd + deg_to_rad<float, float>(STEERING_CAL_OFF),
             2 * M_PI));

    // std::cout << "Final Steering Angle Command (Deg): "
    //           << static_cast<int>(vesc_steering_cmd) << "\n";

    delete cmd;

    uint8_t message[4] = {0, 0, 0, 0};
    int32_t idx = 0;

    CANMessage *cMsg;

    // This block of code only allows position control in between the limit
    // switches.
    // if (!rightLimitSwitch) {

    //   if (vesc_steering_cmd > prev_vesc_steering_cmd) {
    //     buffer_append_uint32(&message[0], vesc_steering_cmd, &idx);
    //     cMsg = new CANMessage(VESC_POSITION_ID(STEER_VESC_ID), &message[0],
    //                           sizeof(message), CANData, CANExtended);
    //   } else {
    //     buffer_append_uint32(&message[0], 0, &idx);
    //     cMsg = new CANMessage(VESC_CURRENT_ID(STEER_VESC_ID), &message[0],
    //                           sizeof(message), CANData, CANExtended);
    //   }
``
    // } else if (!leftLimitSwitch) {

    //   if (vesc_steering_cmd < prev_vesc_steering_cmd) {
    //     buffer_append_uint32(&message[0], vesc_steering_cmd, &idx);
    //     cMsg = new CANMessage(VESC_POSITION_ID(STEER_VESC_ID), &message[0],
    //                           sizeof(message), CANData, CANExtended);
    //   } else {
    //     buffer_append_uint32(&message[0], 0, &idx);
    //     cMsg = new CANMessage(VESC_CURRENT_ID(STEER_VESC_ID), &message[0],
    //                           sizeof(message), CANData, CANExtended);
    //   }

    // } else {
    // prev_vesc_steering_cmd = vesc_steering_cmd;
    buffer_append_int32(&message[0], vesc_steering_cmd * -1000000, &idx);
    // std::cout << "Final Steering Angle Command (Deg): " <<
    // static_cast<int>(vesc_steering_cmd * -1000000) << "\n";
    cMsg = new CANMessage(VESC_POSITION_ID(STEER_VESC_ID), &message[0],
                          sizeof(message), CANData, CANExtended);
    // }

    if(!can_cmd_queue.try_put(cMsg)){
      delete cMsg;
    }

    ThisThread::sleep_for(2ms);
  }
}

/**
 *
 * @brief Implementation of the brake control thread
 * This function is the implementation of the thread that sets the brake
 * possition based on incoming messages from the MRC. The function retrieves the
 * command from the brake command queue, maps it to the appropriate brake output
 * value, and sends the brake command message to the brake motor controller over
 * the CAN bus. The function also clamps the command to prevent it from
 * exceeding the maximum and minimum allowable brake values.
 *
 * The expected incoming message is between 0 and 1 because it means the linear
 * actuator is 0% to 100% exteded. MAX_BRAKE_VAL should be the maximum distance
 * the linear actuator can be extended and any further command won't be able to
 * extend it more. This should vary depending on how the actuator is mounted.
 *
 * The actuator uses a very strange encoding for the message. Here it is
 * summary, but for more details you should look the actuator datasheet. CAN
 * frame format: |       Byte 1      |           Byte 2            | | 8 LSB of
 * position | CE | ME | 5 MSB of position |
 *
 *  CE enable actuator clutch (free move vs. motor engaged)
 *  ME enable actuator motor
 *  5 MSB of position in byte 2 occupies its 5 LSB
 *  Position has a total of 8 + 5 = 13 bits so it has a range of (0 - 8192) and
 * 1000 means 1 inch on the linear actuator
 */

void ActuationController::brake_thread_impl() {
  // Wait for 1 second before starting to allow the rest of the system to
  // initialize
  ThisThread::sleep_for(std::chrono::milliseconds(1000));
  static unsigned char message[8] = {0x0F, 0x4A, 0x00, 0xC0, 0, 0, 0, 0};
  float *cmd;
  uint16_t brake_output = 0;

  while (!ThisThread::flags_get()) // Forever until thread is killed
  {
    brake_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &cmd);
    current_brake_cmd = clamp<float>(*cmd, 0.0, 1.0);
    delete cmd;
    brake_output = map_range<float, uint16_t>(current_brake_cmd, 0.0, 1.0,
                                              MIN_BRAKE_VAL, MAX_BRAKE_VAL);
    message[2] = brake_output & 0xFF; // get the LSB on byte 2
    message[3] = 0xC0 | ((brake_output >> 8) &
                         0x1F); // get the 5 MSB on the right of byte 3 and 0x0C
                                // enable cluthc and actuator motor. Search for
                                // bitwise operators to see how this works.

    CAN_BRAKE.write(CANMessage(
        0x00FF0000, message, 8, CANData,
        CANExtended)); // 0x00FF0000 is the CAN ID for the linear actuator
  }
}

/**
 *
 * @brief Implementation of the sensor poll thread.
 * This function is the implementation of the thread sensor_poll_thread. Its
 * main objective is to read the sensors and update the values of the sensors
 * struct. The sensor struct is later used by the other parts of the
 * ActuatorController class. (steering_pid_thread_impl and populate_reading)
 * Sets the poll interval to DEFAULT_SENSOR_POLL_INTERVAL_MS.
 * It also sets up a filter to receive VESC_STATUS_ID messages from the throttle
 * VESC, which is needed for can.read().
 *
 * The function then enters a loop where it reads the steering angle from the
 * encoder, the steering speed and the motor speed from the VESC.
 *
 * For the steering angle it converts it to radiands, applies a calibration
 * offset and wraps the angle to [0, 2*pi) in order to set the 180 degree point
 * at the neutral steering position by software. If the steering angle has
 * wrapped around, the function updates the number of wraps in the sensors
 * struct.
 *
 * The function then calculates the steering speed using the steering angle
 * difference and time difference.
 *
 * The function reads the motor data from a VESC_STATUS_ID message from the CAN
 * bus and sets the values of the sensors struct.
 *
 * Finally, the function sleeps for the poll interval before repeating the loop.
 *
 * TODO: 1. Check how necessary it is to handle steering angle wrapping
 * around. 2. Revise the has_moved variable to make it less confusing and more
 * robust. 3. Clarify what data it is getting from the VESC.
 * @return void
 */

void ActuationController::sensor_poll_thread_impl() {
  // Declare some variables and wait for 1 second to allow everything to settle
  static long long current_time;
  static long long previous_time;
  bool has_moved = false;
  double radDiff;
  static long timeDiff;
  ThisThread::sleep_for(std::chrono::milliseconds(1000));
  // Set the poll interval to DEFAULT_SENSOR_POLL_INTERVAL_MS. This defines the
  // frecuency the sensors are read.
  static constexpr std::chrono::milliseconds poll_interval{
      DEFAULT_SENSOR_POLL_INTERVAL_MS};
  // Set up a filter to receive VESC_STATUS_ID messages from the throttle VESC.
  // Needed for can.read()
  static const auto throttle_vesc_status_filter = CAN_THROTTLE.filter(
      VESC_STATUS_ID_PACKET_1(THROTTLE_VESC_ID), 0x0001, CANExtended);

  // Set the current and previous time to the current time
  previous_time = us_ticker_read();
  current_time = us_ticker_read();
  while (!ThisThread::flags_get()) // Forever until thread is killed
  {
    static float oldWrap;
    // oldRad is used to calculate the steering speed
    float oldRad = sensors.steering_rad;

    // Get the steering angle from the encoder and convert it to radians
    float newRad = steer_encoder.dutycycle();
    newRad = map_range<float, float>(newRad, 0.0, 1.0, 0.0, 2 * M_PI);
    // Apply a calibration offset and wrap the angle to [0, 2*pi). It is a way
    // to set the 180 degree point at the neutral steering position by software
    newRad =
        fmod(newRad + deg_to_rad<float, float>(STEERING_CAL_OFF), 2 * M_PI);
    // std::cout << newRad << std::endl;

    // Check if the steering angle has wrapped around and update the number of
    // wraps Setting the neutral position to 180 should prevent this from being
    // needed
    // TODO: check how necessary this is
    if ((newRad < M_PI / 4 && sensors.steering_rad > 7 * M_PI / 4) ||
        (sensors.steering_rad < M_PI / 4 && newRad > 7 * M_PI / 4)) {
      if (sensors.steering_wraps == 0)
        oldWrap = newRad;
      if (std::abs(sensors.steering_rad - oldWrap) >
          std::abs(sensors.steering_rad - (oldWrap + 2 * M_PI)))
        sensors.steering_wraps = -1;
      else if (std::abs(sensors.steering_rad - oldWrap) >
               std::abs(sensors.steering_rad - (oldWrap - 2 * M_PI)))
        sensors.steering_wraps = 1;
      else
        sensors.steering_wraps = 0;
    }

    // Set the steering angle in the sensors struct
    sensors.steering_rad = newRad;

    // Calculate the steering speed if the steering has moved
    // TODO: has_moved is confusing. Right now it is only used to skip the speed
    // calculation the first time the loop is run. It could be completetly
    // implemented to detect if it has moved, it could be renamed to a different
    // thing, or the same behaviour could be implemented differently.
    current_time = us_ticker_read();
    if (has_moved) {
      radDiff = oldRad - newRad;
      timeDiff = current_time - previous_time;
      sensors.steering_speed =
          pow(10, 6) * radDiff /
          timeDiff; // pow(10, 6) is there to change the units from us to s
      // std::cout << radSpeed << std::endl;
    }
    if (!has_moved)
      has_moved = true;
    previous_time = current_time;

    // Read the motor data from a VESC_STATUS_ID message from the CAN bus. rr
    // and rl mean rear right and rear left. But I'm not sure if it is reading
    // position of speed from the throttle.
    CANMessage throttle_vesc_status_msg;
    if (CAN_THROTTLE.read(throttle_vesc_status_msg,
                          throttle_vesc_status_filter)) {
      int32_t idx = 0;
      sensors.rl_rad =
          buffer_get_float32(&throttle_vesc_status_msg.data[0], 1.0, &idx);
      sensors.rr_rad = sensors.rl_rad;
    }

    ThisThread::sleep_for(poll_interval);
  }
}
} // namespace gkc
} // namespace tritonai