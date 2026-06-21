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
 * File:        vehicle_class.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Getter and setter functions for the vehicle class object.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "vehicle_class.hpp"

// Getter and setter functions for Pixhawk timestamp
uint64_t vehicle_states::get_pixhawktime() const { return pixtime.load(std::memory_order_acquire); }
void vehicle_states::set_pixhawktime(uint64_t value) { pixtime.store(value, std::memory_order_release); }

// Getter and setter functions for control Node time
double vehicle_states::get_controltime() const { return control_time.load(std::memory_order_acquire); }
void vehicle_states::set_controltime(double value) { control_time.store(value, std::memory_order_release); }

// Getter and setter functions for x
double vehicle_states::get_x() const { return x.load(std::memory_order_acquire); }
void vehicle_states::set_x(double value) { x.store(value, std::memory_order_release); }

// Getter and setter functions for y
double vehicle_states::get_y() const { return y.load(std::memory_order_acquire); }
void vehicle_states::set_y(double value) { y.store(value, std::memory_order_release); }

// Getter and setter functions for z
double vehicle_states::get_z() const { return z.load(std::memory_order_acquire); }
void vehicle_states::set_z(double value) { z.store(value, std::memory_order_release); }

// Getter and setter functions for q0
double vehicle_states::get_q0() const { return q0.load(std::memory_order_acquire); }
void vehicle_states::set_q0(double value) { q0.store(value, std::memory_order_release); }

// Getter and setter functions for q1
double vehicle_states::get_q1() const { return q1.load(std::memory_order_acquire); }
void vehicle_states::set_q1(double value) { q1.store(value, std::memory_order_release); }

// Getter and setter functions for q2
double vehicle_states::get_q2() const { return q2.load(std::memory_order_acquire); }
void vehicle_states::set_q2(double value) { q2.store(value, std::memory_order_release); }

// Getter and setter functions for q3
double vehicle_states::get_q3() const { return q3.load(std::memory_order_acquire); }
void vehicle_states::set_q3(double value) { q3.store(value, std::memory_order_release); }

// Getter and setter functions for vx
double vehicle_states::get_vx() const { return vx.load(std::memory_order_acquire); }
void vehicle_states::set_vx(double value) { vx.store(value, std::memory_order_release); }

// Getter and setter functions for vy
double vehicle_states::get_vy() const { return vy.load(std::memory_order_acquire); }
void vehicle_states::set_vy(double value) { vy.store(value, std::memory_order_release); }

// Getter and setter functions for vz
double vehicle_states::get_vz() const { return vz.load(std::memory_order_acquire); }
void vehicle_states::set_vz(double value) { vz.store(value, std::memory_order_release); }

// Getter and setter functions for rollspeed
double vehicle_states::get_rollspeed() const { return rollspeed.load(std::memory_order_acquire); }
void vehicle_states::set_rollspeed(double value) { rollspeed.store(value, std::memory_order_release); }

// Getter and setter functions for pitchspeed
double vehicle_states::get_pitchspeed() const { return pitchspeed.load(std::memory_order_acquire); }
void vehicle_states::set_pitchspeed(double value) { pitchspeed.store(value, std::memory_order_release); }

// Getter and setter functions for yawspeed
double vehicle_states::get_yawspeed() const { return yawspeed.load(std::memory_order_acquire); }
void vehicle_states::set_yawspeed(double value) { yawspeed.store(value, std::memory_order_release); }

// Getter and setter function for roll
double vehicle_states::get_roll() const { return roll.load(std::memory_order_acquire); }
void vehicle_states::set_roll(double value) { roll.store(value, std::memory_order_release); }

// Getter and setter function for pitch
double vehicle_states::get_pitch() const { return pitch.load(std::memory_order_acquire); }
void vehicle_states::set_pitch(double value) { pitch.store(value, std::memory_order_release); }

// Getter and setter function for yaw
double vehicle_states::get_yaw() const { return yaw.load(std::memory_order_acquire); }
void vehicle_states::set_yaw(double value) { yaw.store(value, std::memory_order_release); }


