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
 * File:        control_algorithm_base.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 25, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Utilities for Controller Base Definition. Class with
 *              fixed members that needs to be utilized with all the functions
 *              in each control algorithm.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef CONTROL_ALGORITHM_BASE_HPP_
#define CONTROL_ALGORITHM_BASE_HPP_

#include "vehicle_class.hpp"
#include "flight_params.hpp"
#include "helper_functions.hpp"
#include "control_config.hpp"

using namespace _utilities_;

namespace _control_algorithm_base_{  

/*******************************************************************************************/
//                                     CLASS DEFINITION
/*******************************************************************************************/

class controller_base
{
    public:
        // Constructor
        controller_base(flight_params* p) : control_input(Eigen::Vector<float, 8>::Zero()), flight_params_ptr(p){}

        // Destructor
        ~controller_base() {}

        // Read parameters
        void read_params([[maybe_unused]] const std::string& jsonFile) {
            // Default implementation or leave empty if to be overridden
        }

        // Initialize initial conditions
        void init() {
            // Default implementation or leave empty if to be overridden
        }

        // Controller Update state and trajectory
        void update([[maybe_unused]] double time,
                    [[maybe_unused]] double x,
                    [[maybe_unused]] double y,
                    [[maybe_unused]] double z,
                    [[maybe_unused]] double vx,
                    [[maybe_unused]] double vy,
                    [[maybe_unused]] double vz,
                    [[maybe_unused]] double q0,
                    [[maybe_unused]] double q1,
                    [[maybe_unused]] double q2,
                    [[maybe_unused]] double q3,
                    [[maybe_unused]] double roll,
                    [[maybe_unused]] double pitch,
                    [[maybe_unused]] double yaw,
                    [[maybe_unused]] double w_x,
                    [[maybe_unused]] double w_y,
                    [[maybe_unused]] double w_z) {
            // Default implementation or leave empty if to be overridden
        }

        // Controller run
        void run([[maybe_unused]] const double time_step_rk4_) {
            // Default implementation or leave empty if to be overridden
        }

        // Getter functions for the thrust values
        float get_t1() const { return control_input(0);}
        float get_t2() const { return control_input(1);}
        float get_t3() const { return control_input(2);}
        float get_t4() const { return control_input(3);}
        float get_t5() const { return control_input(4);}
        float get_t6() const { return control_input(5);}
        float get_t7() const { return control_input(6);}
        float get_t8() const { return control_input(7);}
        
    protected:
        // Thrust values - Control Input from the controller.
        Eigen::Vector<float, 8> control_input;      

        // Instance of the flight_params_ptr to point to the flight_parameter object in
        // flight_main.cpp
        flight_params* flight_params_ptr;      
};

} // namespace _control_algorithm_base_

#endif  // CONTROL_ALGORITHM_BASE_HPP_
