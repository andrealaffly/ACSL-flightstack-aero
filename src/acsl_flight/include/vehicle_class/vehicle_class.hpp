/***********************************************************************************************************************
 * Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        vehicle_class.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header for the vehicle class object. This stores all the data of the states for any vehicle.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef VEHICLE_CLASS_HPP_
#define VEHICLE_CLASS_HPP_

#include <atomic>
#include <cmath>

class vehicle_states {
public:
    // Constructor with default values.
    vehicle_states(uint64_t intial_pix_time = 0, double initital_control_time = 0,
                   double initial_x = 0.0, double initial_y = 0.0, double initial_z = 0.0,
                   double initial_q0 = 1.0, double initial_q1 = 0.0, double initial_q2 = 0.0, double initial_q3 = 0.0,
                   double initial_vx = 0.0, double initial_vy = 0.0, double initial_vz = 0.0,
                   double initial_rollspeed = 0.0, double initial_pitchspeed = 0.0, double initial_yawspeed = 0.0,
                   double initial_roll = 0.0, double initial_pitch = 0.0, double initial_yaw = 0.0) :
                   pixtime(intial_pix_time), control_time(initital_control_time),
                   x(initial_x), y(initial_y), z(initial_z),
                   q0(initial_q0), q1(initial_q1), q2(initial_q2), q3(initial_q3),
                   vx(initial_vx), vy(initial_vy), vz(initial_vz),
                   rollspeed(initial_rollspeed), pitchspeed(initial_pitchspeed), yawspeed(initial_yawspeed),
                   roll(initial_roll), pitch(initial_pitch), yaw(initial_yaw) {}

    // Getter and setter function for pixhawk timestamp
    uint64_t get_pixhawktime() const;
    void set_pixhawktime(uint64_t value);

    // Getter and setter function for contol node time
    double get_controltime() const;
    void set_controltime(double value);

    // Getter and setter functions for x
    double get_x() const;
    void set_x(double value);

    // Getter and setter functions for y
    double get_y() const;
    void set_y(double value);

    // Getter and setter functions for z
    double get_z() const;
    void set_z(double value);

    // Getter and setter functions for q0
    double get_q0() const;
    void set_q0(double value);

    // Getter and setter functions for q1
    double get_q1() const;
    void set_q1(double value);

    // Getter and setter functions for q2
    double get_q2() const;
    void set_q2(double value);

    // Getter and setter functions for q3
    double get_q3() const;
    void set_q3(double value);

    // Getter and setter functions for vx
    double get_vx() const;
    void set_vx(double value);

    // Getter and setter functions for vy
    double get_vy() const;
    void set_vy(double value);

    // Getter and setter functions for vz
    double get_vz() const;
    void set_vz(double value);

    // Getter and setter functions for rollspeed
    double get_rollspeed() const;
    void set_rollspeed(double value);

    // Getter and setter functions for pitchspeed
    double get_pitchspeed() const;
    void set_pitchspeed(double value);

    // Getter and setter functions for yawspeed
    double get_yawspeed() const;
    void set_yawspeed(double value);    

    // Getter and setter function for roll
    double get_roll() const;
    void set_roll(double value);

    // Getter and setter function for pitch
    double get_pitch() const;
    void set_pitch(double value);

    // Getter and setter function for yaw
    double get_yaw() const;
    void set_yaw(double value);    

private:
    // Member variables
    std::atomic<uint64_t> pixtime;      // Timestamp of the data coming in.
    std::atomic<double> control_time;   // Time in the control node.
    std::atomic<double> x;               // position along x-axis in I - NED earth-fixed frame [m]
    std::atomic<double> y;               // position along y-axis in I - NED earth-fixed frame [m]
    std::atomic<double> z;               // position along z-axis in I - NED earth-fixed frame [m]
    std::atomic<double> q0;              // in I - NED earth-fixed frame [-]
    std::atomic<double> q1;              // in I - NED earth-fixed frame [-]
    std::atomic<double> q2;              // in I - NED earth-fixed frame [-]
    std::atomic<double> q3;              // in I - NED earth-fixed frame [-]
    std::atomic<double> vx;              // speed along x-axis in I - NED earth-fixed frame [m/s]
    std::atomic<double> vy;              // speed along y-axis in I - NED earth-fixed frame [m/s]
    std::atomic<double> vz;              // speed along z-axis in I - NED earth-fixed frame [m/s]
    std::atomic<double> rollspeed;       // angular velocity about the x-axis in J - FRD body-fixed frame [rad/s]
    std::atomic<double> pitchspeed;      // angular velocity about the y-axis in J - FRD body-fixed frame [rad/s]
    std::atomic<double> yawspeed;        // angular velocity about the z-axis in J - FRD body-fixed frame [rad/s]
    std::atomic<double> roll;            // Euler Roll  [rad]
    std::atomic<double> pitch;           // Euler Pitch [rad]
    std::atomic<double> yaw;             // Euler Yaw   [rad]
};

#endif  // VEHICLE_CLASS_HPP_