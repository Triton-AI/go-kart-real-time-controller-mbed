/**
 * @file RController.cpp
 * @author Jesus Fausto (jvfausto@ucsd.edu)
 * @brief
 * @version 0.1
 * @date 2022-07-17
 *
 * @copyright Copyright 2022 Triton AI
 * This header file id used for communication with an RC controller.
 * The initial struct defines functions for converting PWM duty cycles into
 * a percentage of maximum throttle. Steering() converts a duty cycle into
 * radians using PI. Trigger() takes a PWM and converts it into the %of max
 * throttle. Red() is a boolean checks to see in an input duty cycle is within a
 * certain range If not, Red() returns false.
 *
 */

#include "Controller/controller.hpp"
#include "PwmIn.h"
#include "config.hpp"
#include "elrs_receiver.hpp"
#include "mbed.h"
#include "mbed_events.h"

#define PI 3.141592654
#define throttlePad 1
#define steerPad 3
#define emoPadLeft 4
#define emoPadRight 7
#define rightTriSwitch 6

namespace tritonai {
namespace gkc {

struct Translation {

  double normalize(int analogValue) {
    if (analogValue > 1800)
      analogValue = 1800;
    if (analogValue < 174)
      analogValue = 174;
    analogValue -= 992;
    return analogValue >= 0 ? (double(analogValue) / (1800 - 992))
                            : (double(analogValue) / (992 - 174));
  }

  double Steering(int steerVal) {
    double Steering_Ang;
    double Steering_Ang_rad;
    double normalized_value = normalize(steerVal);
    if (normalized_value < 0)
      Steering_Ang = -normalized_value * MIN__WHEEL_STEER_DEG;
    else
      Steering_Ang = normalized_value * MAX__WHEEL_STEER_DEG;
    if (Steering_Ang > MAX__WHEEL_STEER_DEG)
      Steering_Ang = MAX__WHEEL_STEER_DEG;
    if (Steering_Ang < MIN__WHEEL_STEER_DEG)
      Steering_Ang = MIN__WHEEL_STEER_DEG;
    if (-.1 < Steering_Ang && Steering_Ang < .1)
      Steering_Ang = 0.0;

    Steering_Ang_rad = Steering_Ang * (PI / 180);

    return Steering_Ang_rad;
  }

  double Throttle(int throttleVal) {
    double normalized_value = normalize(throttleVal);
    // if(normalized_value < .1) normalized_value = 0;
    return normalized_value;
  }

  bool Break(int breakVal) {
    double normalized_value = normalize(breakVal);
    if (normalized_value > -.1)
      normalized_value = 0;
    return normalized_value;
    return 0;
  }

  bool emoVal(int emoVal1, int emoVal2, int rightTriVal) {
    double emoNorm1 = normalize(emoVal1);
    double emoNorm2 = normalize(emoVal2);
    double triNorm = normalize(rightTriVal);
    // if(triNorm<.5&&triNorm>-.5)
    //     return true;
    if (emoNorm1 < 0 && emoNorm2 < 0)
      return false;
    return true;
  }

  bool whoControlls(int rightTriVal) {
    double triNorm = normalize(rightTriVal);
    if (triNorm < 0)
      return false;
    return true;
  }
  /*
  float Steering(float Steering_Duty) {
       //50 degree maximum left and right
       double Steering_Ang;
       double Steering_Ang_rad;
       double a = -(50000/49)*.6;
       double b = -1000*.6;
       if(Steering_Duty <= 0.151) Steering_Ang = b * (Steering_Duty-0.151);
  //this is not correct uses 50 degrees max else if(Steering_Duty >= 0.151)
  Steering_Ang = a * (Steering_Duty-0.151);

      //ensure that Steering angle does not exceed 50 degrees
      //Max and Min steering angles are set to +- 20deg in config.hpp header
  file if(Steering_Ang > MAX__WHEEL_STEER_DEG) Steering_Ang =
  MAX__WHEEL_STEER_DEG; if(Steering_Ang <MIN__WHEEL_STEER_DEG) Steering_Ang =
  MIN__WHEEL_STEER_DEG; if ( -1 < Steering_Ang && Steering_Ang < 1 )
  Steering_Ang = 0.0;

      Steering_Ang_rad = Steering_Ang * (PI/180);

      return Steering_Ang_rad;

      //return Steering_Duty;
  }

  //The trigger function takes in the trigger duty cycle and returns percentage
  of throttle output. float Trigger(float Trigger_Duty){   //2000x-300=y float
  throttle; if (Trigger_Duty <= 0.151) throttle = 0.0;

      else throttle = (2000 * Trigger_Duty) - 300;

      if(throttle > 100) throttle = 100;

      throttle = throttle/100; //added this new line after leaving lab

      return throttle;
  }

  //Red is a boolean function that checks the range of the dutycycle argument
  //if Red_Duty is not between 19% and 25% the return is false
  bool Red(float Red_Duty){
      if (Red_Duty >= 0.19 && Red_Duty < .25) return true;
      else return false;
  }
  */
};

class RCController {
  // The following functions can be used outside the RCController class
public:
  RCController();
  void getSensor();
  // The following attributes(variables) can only be uned in the RCController
  // class
  //
private:
  int noMsgCounter{0};
  float currSteer{0}, currThrottle{0}, currBreak{0};
  Controller *cont_p;
  bool remoteControl{false};
  int emoCounter{0};
  EventQueue sensor_write;
  Translation Map;
};
} // namespace gkc
} // namespace tritonai
