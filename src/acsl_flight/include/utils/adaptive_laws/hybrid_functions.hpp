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
 * File:        hybrid_functions.hpp       \n 
 * Author:      Giri Mugundan Kumar        \n 
 * Date:        February 17, 2025          \n 
 * For info:    Andrea L'Afflitto          \n 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of functions for hybrid MRAC.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged.git
 **********************************************************************************************************************/

 /**
 * @file hybrid_functions.hpp
 * @brief Implementation of functions for hybrid MRAC.
 */
#ifndef HYBRID_FUNCTIONS_HPP_
#define HYBRID_FUNCTIONS_HPP_

#include "helper_functions.hpp"
#include "global_helpers.hpp"

using namespace _utilities_;
using namespace _flightstack_;

// Tolerances and defines for hybrid functionality ---------------------------------------------------------------------
inline constexpr double S_HYBRID_BOUND =  1e9;
inline constexpr double TOLLERANCE_TIME_RESET_SERIES = 5e-1;

/**********************************************************************************************************************/
/*                                          HYBRID VARIABLES FOR EASY ACCESS                                          */
/**********************************************************************************************************************/
// Hybrid variables that can be intialized easily ----------------------------------------------------------------------
namespace _hybrid_variables_
{

template <typename Scalar, int n, int m>
struct hybrid_internal_members
{
  
  Scalar e_transpose_Q_e;
  Scalar summation_P;
  Scalar s;
  Scalar s_element;
  Scalar reset_time;
  bool summation_condition_hybrid_verified_flag;
  bool hybrid_reset_condition_verified_flag;
  
  Eigen::Matrix<Scalar, n, m> e_prev;
  Eigen::Matrix<Scalar, n, m> jump_map;
  
};

template <typename Scalar, int n, int m>
struct hybrid_integrated_state_members
{
  Scalar e_transpose_Q_e_I;
};


template <typename Scalar, int n, int m>
struct hybrid_internal_parameters
{
  Scalar alpha;
};



} // namespace _hybrid_variables_


/**********************************************************************************************************************/
/*                                          HYBRID FUCNTIONS FOR EASY ACCESS                                          */
/**********************************************************************************************************************/
namespace _hybrid_functions_
{ 

// Function to compute the output of the series
template <typename T>
inline T series_element(T s, T alpha)
{
  /**
   * Computes the s-th term in the series used to determine reference resetting events.
   * See Algorithm 2 - (1) in Adaptive control for hybrid dynamical systems with user-defined
   * rate of convergence - Mattia Gramuglia, Giri Mugundan Kumar, Andrea L'Afflitto.
   *
   * @param s The index of the term in the series.
   * @param alpha \alpha > 1.
   * @return The computed series element.
   */
  return static_cast<T>(1.0) / std::pow(s, alpha);
}

// Function to compute s such that \sum s is convergent
template <typename T>
inline void find_s(T& s, T weighted_e_squared, T alpha)
{
  /**
   * This function computes s such that the sum of s is convergent.
   * See Algorithm 2 - (2) in Adaptive control for hybrid dynamical systems with user-defined
   * rate of convergence - Mattia Gramuglia, Giri Mugundan Kumar, Andrea L'Afflitto.
   *
   * weighted_e_squared = e^T(t_j) P e(t_j)
   * 
   * @param s_prev The previous index of the term in the series.
   * @param alpha \alpha > 1.
   * @return The computed series element.
   */

  // cache the new value for s in a temp variable
  T temp = std::max(std::pow(weighted_e_squared, (-static_cast<T>(1.0) / alpha)), s + static_cast<T>(1.0));

  // in-place opertation for updating the series
  s = temp;
}

// Function that updates the hybrid summation
template <typename Scalar, int n, int m>
inline void update_hybrid_P_summation(const Eigen::Matrix<Scalar, n, m>& e, 
                                      const Eigen::Matrix<Scalar, n, n>& P, 
                                      _hybrid_variables_::hybrid_internal_members<Scalar, n, m>& him,
                                      const std::optional<std::string>& msg = std::nullopt)
{
    Scalar current_value = (e.transpose() * P * e)(0,0);                      // <---- (0,0) extracts the double
    Scalar previous_value = (him.e_prev.transpose() * P * him.e_prev)(0,0);   // <---- (0,0) extracts the double

    // Ensure compatibility of matrix dimensions
    assert(e.rows() == n && e.cols() == m);
    assert(P.rows() == n && P.cols() == n);

    if (current_value > previous_value)
    {
        him.summation_P += (current_value - previous_value);
        him.summation_condition_hybrid_verified_flag = true;
    }

    // Print message if provided
    if (msg)
    {
        FLIGHTSTACK_INFO(*msg);
    }
}

// Function that resets the hybrid boolean flags
template <typename Scalar, int n, int m>
inline void reset_hybrid_flags(_hybrid_variables_::hybrid_internal_members<Scalar, n, m>& him)
{
  // Reset the boolean flag for summation condition on P
  him.summation_condition_hybrid_verified_flag = false;

  // Reset the boolean flag for the hybrid reset condition
  him.hybrid_reset_condition_verified_flag = false;

  // Reset the jump map to zero
  him.jump_map << Eigen::Matrix<Scalar, n, m>::Zero();

}

// Function that checks the reset condition
template<typename Scalar, int n, int m>
inline bool check_hybrid_reset_cond(const double& current_time, _hybrid_variables_::hybrid_internal_members<Scalar, n, m>& him,
                                    const std::optional<std::string>& msg = std::nullopt)
{
  if (him.s > S_HYBRID_BOUND || (him.s > 0 && (current_time - him.reset_time > TOLLERANCE_TIME_RESET_SERIES)))
  {
    // Reset s_series
    him.s = 0.0;

    // Print message if provided
    if (msg)
    {
        FLIGHTSTACK_INFO(*msg);
    }

    return true;  // Indicate that the reset condition is met
  }
  return false;   // If the condition is not met
}


// Function that handles the resetting event 
template <typename Scalar, int n, int m>
inline void hybrid_reset_event(const double& current_time, const Eigen::Matrix<Scalar, n, m>& e, 
                               const Eigen::Matrix<Scalar, n, n>& P, Eigen::Matrix<Scalar, n, m>& x_ref_traj,
                               _hybrid_variables_::hybrid_internal_members<Scalar, n, m>& him,
                               _hybrid_variables_::hybrid_integrated_state_members<Scalar, n, m>& his,
                               _hybrid_variables_::hybrid_internal_parameters<Scalar, n, m>& hip,
                              const std::optional<std::string>& msg = std::nullopt)
{
  if (his.e_transpose_Q_e_I >= him.summation_P)
  {
    // Update the resetting time to current time
    him.reset_time = current_time;

    // Cache e^T * P * e
    Scalar ePe = (e.transpose() * P * e)(0,0);                        // <-- extracts the component
    Scalar ePe_prev = (him.e_prev.transpose() * P * him.e_prev)(0,0);  // <-- extracts the component

    // Update the series s
    find_s(him.s, ePe, hip.alpha);

    // Find the series element
    him.s_element = series_element(him.s, hip.alpha);

    // Find the jump map
    him.jump_map =  ( 1 - std::sqrt( ( ePe - him.s_element ) / ePe) ) * e;

    // Compute the new /int e * Q * e 
    Scalar update_eQe_I = his.e_transpose_Q_e_I + (ePe - ePe_prev);

    // Update the hybrid variable with the new /int e * Q * e
    his.e_transpose_Q_e_I = update_eQe_I;

    // Compute the new reference trajectory = old reference trajectory + jump map
    Eigen::Matrix<Scalar, n, m> update_traj = x_ref_traj + him.jump_map;

    // Updat the reference traejctory 
    x_ref_traj << update_traj;    
    
    // Print message if provided
    if (msg)
    {
        FLIGHTSTACK_INFO(*msg);
    }
  }
}

// Function that checks the hybrid time dependant resetting events
template <typename Scalar, int n, int m>
inline void hybrid_main_logic(const double& current_time, const Eigen::Matrix<Scalar, n, m>& e, 
                              const Eigen::Matrix<Scalar, n, n>& P, Eigen::Matrix<Scalar, n, m>& x_ref_traj,
                              _hybrid_variables_::hybrid_internal_members<Scalar, n, m>& him,
                              _hybrid_variables_::hybrid_integrated_state_members<Scalar, n, m>& his,
                              _hybrid_variables_::hybrid_internal_parameters<Scalar, n, m>& hip,
                              const std::string& msg)
{
  // Initiate a static boolean that will be changed on the first at the very top of this function
  static bool first_run_flag = true;   // <-- state of this boolean will be kept in memory

  // Logic to avoid processing the hybrid main logic on the first run
  if (first_run_flag) { first_run_flag  = false; return; } // <-- skip execution on first call

  // Call the series conditions
  him.hybrid_reset_condition_verified_flag = check_hybrid_reset_cond(current_time, him, msg + "RESETTING SERIES");

  // Call the trajectory reset conditions
  hybrid_reset_event(current_time, e, P, x_ref_traj, him, his, hip, msg + "RESETTING EVENT SUCCESSFUL");
  
}

} // namespace _hybrid_functions_

#endif  // HYBRID_FUNCTIONS_HPP_