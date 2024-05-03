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
    double Translation::throttle_ratio(int throttleVal) 
    {
        double normalized_value = normalize(throttleVal);
        return (normalized_value+1.0)/2.0;
    }
    bool Translation::keep_constant_thr(int throttleVal) 
    {
        double normalized_value = normalize(throttleVal);
        if (normalized_value < 0.0)
            return false;
        else
            return true;
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

        // TODO: (Moises) Check if the switch is in the correct position
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
        const uint16_t *busData = _receiver.busData();
        while (true)
        {  
            ThisThread::sleep_for(10ms);

            inc_count(); // Increment the rolling counter

            if (!_receiver.gatherData()) continue; // Stop if no data available
            

            if(!_receiver.messageAvailable) continue; // Stop if no message available

            // std::cout << "Bus data: " << (int)(100*busData[0]) <<
            //     " " << (int)(100*busData[1]) <<
            //     " " << (int)(100*busData[2]) <<
            //     " " << (int)(100*busData[3]) <<
            //     " " << (int)(100*busData[4]) <<
            //     " " << (int)(100*busData[5]) <<
            //     " " << (int)(100*busData[6]) <<
            //     " " << (int)(100*busData[7]) <<
            //     " " << (int)(100*busData[8]) <<
            //     std::endl;

            // Stop if the emergency stop is not active
            bool temp_active = Map.is_active(
                busData[ELRS_EMERGENCY_STOP_LEFT],
                busData[ELRS_EMERGENCY_STOP_RIGHT]
            );

            if(!temp_active){
                _packet.throttle = 0.0;
                _packet.steering = 0.0;
                _packet.brake = Map.throttle_ratio(busData[ELRS_RATIO_THROTTLE]);
                _packet.is_active = temp_active;
                _packet.publish(*_sub);
                continue;
            }

            bool keep_constant_thr = Map.keep_constant_thr(busData[ELRS_HOLD_THROTTLE]);
            bool is_all_zero = abs(100*Map.normalize(busData[ELRS_THROTLE])) <= 5 && abs(100*Map.normalize(busData[ELRS_STEERING])) <= 5;


            if(is_all_zero && !keep_constant_thr){
                _packet.throttle = 0.0;
                _packet.steering = 0.0;
                _packet.brake = 0.0;
                _packet.is_active = temp_active;
                _packet.autonomy_mode = Map.getAutonomyMode(
                    busData[ELRS_TRI_SWITCH_RIGHT]
                );
                _packet.publish(*_sub);
                continue;
            }

            if(!keep_constant_thr){
                current_throttle = Map.throttle(busData[ELRS_THROTLE]);
            }

            _packet.throttle = current_throttle*Map.throttle_ratio(busData[ELRS_RATIO_THROTTLE]);
            _packet.brake = 0.0; // TODO: (Moises) Implement brake
            _packet.steering = Map.steering(busData[ELRS_STEERING]);
            _packet.autonomy_mode = Map.getAutonomyMode(
                busData[ELRS_TRI_SWITCH_RIGHT]
            );
            _packet.is_active = temp_active;

            _is_ready = true;

            _packet.publish(*_sub);
        }
    }

    RCController::RCController(GkcPacketSubscriber *sub) :
        Watchable(DEFAULT_RC_CONTROLLER_POLL_INTERVAL_MS, DEFAULT_RC_CONTROLLER_POLL_LOST_TOLERANCE_MS, "RCController"),
        _receiver(REMOTE_UART_RX_PIN,REMOTE_UART_TX_PIN),
        _is_ready(false),
        _sub(sub)
    {
        _rc_thread.start(callback(this, &RCController::update));
        attach(callback(this, &RCController::watchdog_callback));
    }

    void RCController::watchdog_callback()
    {
        std::cout << "RCController watchdog triggered" << std::endl;
        NVIC_SystemReset();
    }
} // namespace tritonai::gkc