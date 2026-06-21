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
 * File:        control_config.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        January 20, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Just a header file for configuring some of the features of the controller. 
 *              >> All controllers use this <<. Also used by the control.cpp file.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef CONTROL_CONFIG_HPP_
#define CONTROL_CONFIG_HPP_

namespace _config_param_
{
  // Flag for enable/disable publishing data to motors
  inline constexpr bool PUBLISH_ACTUATOR  = true;

  // Flag to enable/disable the MRAC Projection Operator
  inline constexpr bool USE_PROJECTION_OPERATOR = true;

  // Flag to enable/disable using a sine wave function for 
  // the inner loop desired orientation.
  inline constexpr bool USE_SINEWAVE_INL_CMD = false;

}

#endif