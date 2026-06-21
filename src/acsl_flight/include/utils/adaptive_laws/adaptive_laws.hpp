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
 * File:        adaptive_laws.hpp       \n 
 * Author:      Giri Mugundan Kumar     \n 
 * Date:        January 17, 2025        \n 
 * For info:    Andrea L'Afflitto       \n 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of various adaptive laws.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged.git
 **********************************************************************************************************************/

/**
 * @file adaptive_laws.hpp
 * @brief Implementation of adaptive laws.
 */
#ifndef ADAPTIVE_LAWS_HPP_
#define ADAPTIVE_LAWS_HPP_

#include "helper_functions.hpp"
#include "deadzone_opertator.hpp"
#include "projection_operator.hpp"

using namespace _utilities_;
using namespace _deadzone_operator_;
using namespace _projection_operator_;

namespace _adaptive_laws_
{

// Classical MRAC Adaptive Laws.
template <typename Der1, typename Der2, typename Der3>
inline auto AdaptiveLawClassical(const Der1& Gamma,
                                 const Der2& pi_vec,
                                 const Der3& eTPB)
{
  // Adaptive law
  auto K_hat_dot = Gamma * pi_vec * eTPB;

  // Return the derivative of the adaptive gain
  return K_hat_dot;
}                                  


// Robust MRAC - Adaptive law formula equipped with the dead-zone modification and e-modificiation
template <typename Der1, typename Der2, typename Der3, typename Der4>
inline auto AdaptiveLawDeadZoneEMod(const Der1& Gamma,
                                    const double dead_zone_value,
                                    const Der2& pi_vec,
                                    const Der3& eTPB,
                                    const double sigma_gain,
                                    const Der4& K_hat_state) 
{
  // Compute the norm of the e'PB
  auto eTPB_norm = eTPB.norm();

  // Compute the adaptive laws
  auto K_hat_dot = Gamma * dead_zone_value * ( pi_vec * eTPB - sigma_gain * eTPB_norm * K_hat_state);

  // Return the derivative of the adaptive gain
  return K_hat_dot;
}
                        

} // _adaptive_laws_


#endif  // ADAPTIVE_LAWS_HPP_