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
 * File:        user_defined_trajectory_base.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 27, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: User Defined Trajectory Base Definition. Class with fixed 
 *              members that needs to be utilized with all the functions
 *              in each user defined trajectory.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef USER_DEFINED_TRAJECTORY_BASE_HPP_
#define USER_DEFINED_TRAJECTORY_BASE_HPP_

#include <Eigen/Dense>                   // Include the Eigen library
#include "helper_functions.hpp"          // Include the helper functions library for the polynomial methods

using namespace Eigen;
using namespace _utilities_;

namespace _user_defined_trajectory_base_{

class trajectory_base
{
    public:

        // Constructor 
        trajectory_base() : ud_pos(Vector3d::Zero()),
                            ud_vel(Vector3d::Zero()),
                            ud_acc(Vector3d::Zero()),
                            ud_angle(Vector3d::Zero()),
                            ud_angular_rate(Vector3d::Zero()),
                            ud_angular_rate_dot(Vector3d::Zero()),
                            ud_angular_vel(Vector3d::Zero()),
                            ud_angular_acc(Vector3d::Zero()){}

        // Getter and setter functions for user_defined_position
        inline const Vector3d& getUserDefinedPosition() const { return ud_pos; }
        inline void setUserDefinedPosition(const Vector3d& pos) { ud_pos = pos; }

        // Getter and setter functions for user_defined_velocity
        inline const Vector3d& getUserDefinedVelocity() const { return ud_vel; }
        inline void setUserDefinedVelocity(const Vector3d& vel) { ud_vel = vel; }

        // Getter and setter functions for user_defined_acceleration
        inline const Vector3d& getUserDefinedAcceleration() const { return ud_acc; }
        inline void setUserDefinedAcceleration(const Vector3d& acc) { ud_acc = acc; }

        // Getter and setter functions for user_defined_angle
        inline const Vector3d& getUserDefinedAngle() const { return ud_angle; }
        inline void setUserDefinedAngle(const Vector3d& angle) { ud_angle = angle; }
        
        // Setter and getter function for individual angles
        inline void setUserDefinedRoll(double x) { ud_angle.x() = x; }
        inline void setUserDefinedPitch(double y) { ud_angle.y() = y; }
        inline void setUserDefinedYaw(double z) { ud_angle.z() = z; }
        inline double getUserDefinedRoll() const { return ud_angle.x(); }
        inline double getUserDefinedPitch() const { return ud_angle.y(); }
        inline double getUserDefinedYaw() const { return ud_angle.z(); }

        // Getter and setter functions for user_defined_angular_rate
        inline const Vector3d& getUserDefinedAngularRate() const { return ud_angular_rate; }
        inline void setUserDefinedAngularRate(const Vector3d& angular_rate) { ud_angular_rate = angular_rate; }

        // Setter and getter function for individual angular rates
        inline void setUserDefinedRollRate(double x) { ud_angular_rate.x() = x; }
        inline void setUserDefinedPitchRate(double y) { ud_angular_rate.y() = y; }
        inline void setUserDefinedYawRate(double z) { ud_angular_rate.z() = z; }
        inline double getUserDefinedRollRate() const { return ud_angular_rate.x(); }
        inline double getUserDefinedPitchRate() const { return ud_angular_rate.y(); }
        inline double getUserDefinedYawRate() const { return ud_angular_rate.z(); }

        // Getter and setter functions for user_defined_angular_rate_dot
        inline const Vector3d& getUserDefinedAngularRateDot() const { return ud_angular_rate_dot; }
        inline void setUserDefinedAngularRateDot(const Vector3d& ang_rt_dot) { ud_angular_rate_dot = ang_rt_dot; }

        // Setter and getter function for individual angular rate dot
        inline void setUserDefinedRollRateDot(double x) { ud_angular_rate_dot.x() = x; }
        inline void setUserDefinedPitchRateDot(double y) { ud_angular_rate_dot.y() = y; }
        inline void setUserDefinedYawRateDot(double z) { ud_angular_rate_dot.z() = z; }
        inline double getUserDefinedRollRateDot() const { return ud_angular_rate_dot.x(); }
        inline double getUserDefinedPitchRateDot() const { return ud_angular_rate_dot.y(); }
        inline double getUserDefinedYawRateDot() const { return ud_angular_rate_dot.z(); }

        // Getter and setter functions for user_defined_angular_velocity
        inline const Vector3d& getUserDefinedAngularVelocity() const { return ud_angular_vel; }
        inline void setUserDefinedAngularVelocity(const Vector3d& angular_vel) { ud_angular_vel = angular_vel; }

        // Setter and getter function for individual angular velocity
        inline void setUserDefinedOmegaX(double x) { ud_angular_vel.x() = x; }
        inline void setUserDefinedOmegaY(double y) { ud_angular_vel.y() = y; }
        inline void setUserDefinedOmegaZ(double z) { ud_angular_vel.z() = z; }
        inline double getUserDefinedOmegaX() const { return ud_angular_vel.x(); }
        inline double getUserDefinedOmegaY() const { return ud_angular_vel.y(); }
        inline double getUserDefinedOmegaZ() const { return ud_angular_vel.z(); }

        // Getter and setter functions for user_defined_angular_acceleration
        inline const Vector3d& getUserDefinedAngularAcceleration() const { return ud_angular_acc; }
        inline void setUserDefinedAngularAcceleration(const Vector3d& angular_acc) { ud_angular_acc = angular_acc; }

        // Setter and getter function for individual angular acceleration
        inline void setUserDefinedAlphaX(double x) { ud_angular_acc.x() = x; }
        inline void setUserDefinedAlphaY(double y) { ud_angular_acc.y() = y; }
        inline void setUserDefinedAlphaZ(double z) { ud_angular_acc.z() = z; }
        inline double getUserDefinedAlphaX() const { return ud_angular_acc.x(); }
        inline double getUserDefinedAlphaY() const { return ud_angular_acc.y(); }
        inline double getUserDefinedAlphaZ() const { return ud_angular_acc.z(); }

    protected:

        // Encapusulated members of the trajectory. 
        // It is set up in a way that you can provide a twice differentiable trajectory. 
        // If you want to add more you can do so.

        // x, y, z in the inertial frame
        Vector3d ud_pos;   

        // vx, vy, vz in the inertial frame
        Vector3d ud_vel;

        // \ddot{x}, \ddot{y}, \ddot{z} in the inertial frame
        Vector3d ud_acc;

        // \phi, \theta, \psi w.r.t inertial frame
        Vector3d ud_angle;

        // \dot{\phi}, \dot{\theta}, \dot{\psi} w.r.t. inertial frame 
        Vector3d ud_angular_rate;

        // \ddot{\phi}, \ddot{\theta}, \ddot{\psi} w.r.t. inertial frame
        Vector3d ud_angular_rate_dot;

        // \omega_x, \omega_y, \omega_z in the body frame
        Vector3d ud_angular_vel;

        // \alpha_x, \alpha_y, \alpha_z in the body frame
        Vector3d ud_angular_acc;

};


} // namespace _user_defined_trajectory_base_

#endif // USER_DEFINED_TRAJECTORY_BASE_HPP_