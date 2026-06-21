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
 * File:        deadzone_operator.hpp \n 
 * Author:      Mattia Gramuglia      \n 
 * Date:        September 4, 2024     \n 
 * For info:    Andrea L'Afflitto     \n 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the smooth version of the dead-zone modification of MRAC through a 
 *              modulation function. For reference: E. Lavretsky, K. Wise, "Robust and Adaptive Control", 
 *              Springer 2013, Sec. 11.2.1
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged.git
 **********************************************************************************************************************/

/**
 * @file projection_operator.hpp
 * @brief Implementation of the deadzone operator to be used in the MRAC controllers.
 */
#ifndef DEADZONE_OPERATOR_HPP
#define DEADZONE_OPERATOR_HPP

#include <Eigen/Dense>

/**
 * For reference: E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.2.1
 */

namespace _deadzone_operator_
{

/*
    Implementation of the smooth version of the dead-zone modification of MRAC through a 
    modulation function
    For reference: E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.2.1
    PARAMETERS:
      - delta: constant that must be 0 < delta < 1. delta characterizes the slope of the modulation function.
      - e_0: constant that must be e_0 > 0. The dead-zone modification stops the adaptation process when 
             the norm of the tracking error becomes smaller than the prescribed value e_0.
  */
template <typename T>
inline double deadZoneModulationFunction(const T& e_vector,
                                         const double& delta,
                                         const double& e_0)
{
  return std::max(0.0, std::min(1.0, (e_vector.norm() - delta * e_0)/((1.0 - delta) * e_0)));
}

} // namespace _deadzone_operator_

#endif // DEADZONE_OPERATOR_HPP