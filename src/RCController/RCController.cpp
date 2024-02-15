#include "RCController/RCController.hpp"
#include <iostream>
#include <string>

namespace tritonai::gkc 
{
    // TRANSLATION
    double Translation::normalize(int analogValue) 
    {
        if (analogValue > 1800)
            analogValue = 1800;
        if (analogValue < 174)
            analogValue = 174;
        analogValue -= 992;
        return analogValue >= 0 ? (double(analogValue) / (1800 - 992))
                                : (double(analogValue) / (992 - 174));
    }
    double Translation::throttle(int throttleVal) 
    {
        return normalize(throttleVal);
    }
    double Translation::brake(int brakeVal) 
    {
        return normalize(brakeVal);
    }
    double Translation::steering(int steerVal) 
    {
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

        Steering_Ang_rad = Steering_Ang * (M_PI / 180);

        return Steering_Ang_rad;
    }
    bool Translation::is_active(int swith1, int swith2) 
    {
        double swith1_norm = normalize(swith1);
        double swith2_norm = normalize(swith2);

        // TODO: Check if the switch is in the correct position
        if (swith1_norm < 0.0 && swith2_norm < 0.0)
            return true;
        else
            return false;
    }
    AutonomyMode Translation::getAutonomyMode(int rightTriVal)
    {
        double rightTriVal_norm = normalize(rightTriVal);
        if (rightTriVal_norm < -0.5)
            return AUTONOMOUS;
        else if (rightTriVal_norm > 0.5)
            return MANUAL;
        else
            return AUTONOMOUS_OVERRIDE;
    }

    // RCController  
    void RCController::update() 
    {
        while (true)
        {   
            ThisThread::sleep_for(100ms);

            if (!_receiver.gatherData()) continue; // Stop if no data available
            

            if(!_receiver.messageAvailable) continue; // Stop if no message available

            const uint16_t *busData = _receiver.busData();


            _packet.throttle = Map.throttle(busData[ELRS_THROTLE]);
            _packet.brake = 0.0; // TODO: Implement brake
            _packet.steering = Map.steering(busData[ELRS_STEERING]);
            _packet.is_active = Map.is_active(
                busData[ELRS_EMERGENCY_STOP_LEFT],
                busData[ELRS_EMERGENCY_STOP_RIGHT]
            );
            _packet.autonomy_mode = Map.getAutonomyMode(
                busData[ELRS_TRI_SWITCH_RIGHT]
            );
        }
    }

    RCController::RCController() :
        _receiver(REMOTE_UART_RX_PIN,REMOTE_UART_TX_PIN)
    {
        _rc_thread.start(callback(this, &RCController::update));
        std::cout << "RCController created" << std::endl;
    }
} // namespace tritonai::gkc