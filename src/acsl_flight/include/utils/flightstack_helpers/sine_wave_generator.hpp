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
 * File:        sine_wave_generator.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        January 30, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Inline function to generate a sine wave. Used to tune the reference model for the inner loop
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef SINE_WAVE_GENERATOR_
#define SINE_WAVE_GENERATOR_

#include <cmath>

/// ------------- USER DEFINED ------------- ///
// - > The amplitude and frequency for the sine generator test function
inline constexpr double AMP = 5*(M_PI/180);
inline constexpr double FREQ = 0.5;        

namespace _test_functions_
{

// Function generates a sine wave of formula A*sin(w*t) for the desired orientations and 
// the rate of change A*w*cos(w*t) for the desired yaw rate. This is to tune the reference
// model of the inner loop.
inline void generateSineWave(double time, double &roll_d, double &pitch_d, double &yaw_d, double &yaw_rate_d) 
{
    double value = AMP * std::sin(2 * M_PI * FREQ * time);
    double rate_of_change = 2 * M_PI * AMP * FREQ * std::cos(2 * M_PI * FREQ * time);
    
    roll_d = value;
    pitch_d = value;
    yaw_d = value;
    yaw_rate_d = rate_of_change;
}

} // namespace _test_functions_


#endif // SINE_WAVE_GENERATOR_