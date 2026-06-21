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
 * File:        MRAC_PID_logger.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 26, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: logging for the MRAC_PID controller. Inherits the Blackbox 
 *              class for logging and takes in the internal members for 
 *              MRAC_PID.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

/*
________/\\\__________/\\\\\\\\\______/\\\\\\\\\\\\\____/\\\\\\\\\\\\\___        
 _____/\\\\/\\\\_____/\\\///////\\\___\/\\\/////////\\\_\/\\\/////////\\\_       
  ___/\\\//\////\\\__\/\\\_____\/\\\___\/\\\_______\/\\\_\/\\\_______\/\\\_      
   __/\\\______\//\\\_\/\\\\\\\\\\\/____\/\\\\\\\\\\\\\\__\/\\\\\\\\\\\\\/__     
    _\//\\\______/\\\__\/\\\//////\\\____\/\\\/////////\\\_\/\\\/////////____    
     __\///\\\\/\\\\/___\/\\\____\//\\\___\/\\\_______\/\\\_\/\\\_____________   
      ____\////\\\//_____\/\\\_____\//\\\__\/\\\_______\/\\\_\/\\\_____________  
       _______\///\\\\\\__\/\\\______\//\\\_\/\\\\\\\\\\\\\/__\/\\\_____________ 
        _________\//////___\///________\///__\/////////////____\///______________
*/

#ifndef CONTROLLERS_MRAC_PID_LOGGER_HPP_
#define CONTROLLERS_MRAC_PID_LOGGER_HPP_

#include "MRAC_PID_members.hpp"       // Include for all the members of the PID class
#include "flight_log.hpp"             // Include for the virtual class for logging
#include "global_helpers.hpp"         // Include for the logging functionality

using namespace _flightstack_;

using namespace _flight_log_;

namespace _qrbp_{
namespace _mrac_pid_{

class mrac_pid_logger : public blackbox
{
public:
    // Constructor
    mrac_pid_logger(controller_internal_members* cim, 
                    controller_integrated_state_members* csm,
                    Eigen::Vector<float, 8>* control_input,
                    const std::string & controller_log_dir_);

    // Implementing functions from blackbox
    void logInitHeaders();
    bool logInitLogging();
    void logLogData();

private:
    // Pointers to store the address of internal members of the controller
    controller_internal_members* CIM;   

    // Pointers to store the address of internal integrated state members of the controller
    controller_integrated_state_members* CSM; 

    // Control input vector
    Eigen::Vector<float, 8>* cntrl_input;

    // Global Log directory
    std::string flight_run_log_directory;
};

} // namespace _mrac_pid_
} // namespace _qrbp_

#endif  // CONTROLLERS_PID_LOGGER_HPP_