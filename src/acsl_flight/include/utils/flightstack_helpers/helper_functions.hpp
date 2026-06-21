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
 * File:        helper_functions.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        May 29, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Definitions of some standard templates used in all 
 *              controllers.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef HELPER_FUNCTIONS_HPP_
#define HELPER_FUNCTIONS_HPP_

#include <array>                         // Include the header for std::array
#include <Eigen/Dense>                   // Include the Eigen library
#include <nlohmann/json.hpp>             // Include the nlohmann JSON library
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>           // Include the correct header for bind
#include <boost/bind/placeholders.hpp>   // Include the placeholders
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/interval.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/math/tools/polynomial.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <time.h>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <continuous_lyapunov_equation.hpp>
#include <tuple>
#include <optional>
#include <sine_wave_generator.hpp>

using namespace Eigen;
namespace bph = boost::placeholders; // Use the correct namespace
using namespace _lyapunov_solver_;

namespace _utilities_
{
    // Function to evaluate a polynomial given its coefficients and a value at which to evaluate the polynomial
    // Example usage: double value = 3.0;
    //                Eigen::VectorXd coeffs(3);
    //                coeffs << 1, -2, 1; // Represents the polynomial x^2 - 2x + 1
    //                double result = evaluatePolynomial(coeffs, value);
    template <typename VectorType, typename ScalarType>
    inline ScalarType evaluatePolynomial(const VectorType coefficients, ScalarType value) {
        ScalarType result = 0;
        for (int i = 0, deg = coefficients.size() - 1; deg >= 0; deg--, i++)
        {
            result += coefficients[i] * std::pow(value, deg);
        }
        return result;
    }

    // Compute the 2D norm using polynomial coefficients
    // Example usage: poly_coef_x << 1, -2, 1;  // Example coefficients for x
    //                poly_coef_y << 2, 0, -1;  // Example coefficients for y
    //                double t = 1.0;  // Example value for t
    //                double result = norm2D<Eigen::VectorXd, double>(poly_coef_x, poly_coef_y, t);
    template<typename VectorType, typename ScalarType>
    inline ScalarType norm2D(const VectorType& poly_coef_x, const VectorType& poly_coef_y, ScalarType value)
    {
        ScalarType norm_value = std::sqrt(std::pow(evaluatePolynomial(poly_coef_x, value), 2) +
                                        std::pow(evaluatePolynomial(poly_coef_y, value), 2));
        return norm_value;
    }

    // Compute the derivative of the 2D norm using polynomial coefficients and their derivatives
    // Example Usage: poly_coef_x << 1, -2, 1;  // Example coefficients for x
    //                poly_coef_y << 2, 0, -1;  // Example coefficients for y
    //                poly_coef_x_prime << 2, -2, 0;  // Example coefficients for x'
    //                poly_coef_y_prime << 0, 0, -1;  // Example coefficients for y'
    //                double t = 1.0;  // Example value for t
    //                double result = norm2Dderivative<Eigen::VectorXd, double>(poly_coef_x, 
    //                                                                          poly_coef_y,
    //                                                                          poly_coef_x_prime,
    //                                                                          poly_coef_y_prime,
    //                                                                          t);
    template<typename VectorType, typename ScalarType>
    inline ScalarType norm2Dderivative(const VectorType& poly_coef_x,
                                    const VectorType& poly_coef_y,
                                    const VectorType& poly_coef_x_prime,
                                    const VectorType& poly_coef_y_prime,
                                    ScalarType value)
    {
        ScalarType norm_value = norm2D<VectorType, ScalarType>(poly_coef_x, poly_coef_y, value);
        ScalarType derivative_value = ((evaluatePolynomial<VectorType, ScalarType>(poly_coef_x, value) *
                                        evaluatePolynomial<VectorType, ScalarType>(poly_coef_x_prime, value) +
                                        evaluatePolynomial<VectorType, ScalarType>(poly_coef_y, value) *
                                        evaluatePolynomial<VectorType, ScalarType>(poly_coef_y_prime, value)) /
                                        norm_value);
        return derivative_value;
    }

    // Function to compute the derivative of the piecewise polynomial coefficient matrix
    // Example Usage: Eigen::MatrixXd poly_coeff_matrix_in(3, 4);
    //                Eigen::MatrixXd result = polyDerMatrix(poly_coeff_matrix_in);
    template<typename MatrixType>
    inline MatrixType polyDerMatrix(const MatrixType& poly_coeff_matrix_in)
    {
        // Get the size of the input matrix
        int rows = poly_coeff_matrix_in.rows();
        int cols = poly_coeff_matrix_in.cols();
        
        // Initialize the output matrix for the derivative
        MatrixType poly_coeff_matrix_out(rows, cols - 1);
        
        // Compute the derivative for each row of the input matrix
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols - 1; ++j) {
                poly_coeff_matrix_out(i, j) = (cols - 1 - j) * poly_coeff_matrix_in(i, j);
            }
        }
        
        return poly_coeff_matrix_out;
    }


    // Wraps an angle `alpha` to the interval [-pi, pi]
    template <typename T>
    inline T wrapAngleToMinusPiAndPi(T alpha)
    {
        return alpha - T(2 * M_PI) * std::floor((alpha + T(M_PI)) / (T(2 * M_PI)));
    }

    // Makes the yaw angular error continuous considering the discontinuity at +/- pi
    // Example usage :  double result = makeYawAngularErrorContinuous<double>(yaw, user_defined_yaw);
    // Example usage :  float result = makeYawAngularErrorContinuous<float>(yaw_float, user_defined_yaw_float);
    template <typename T>
    inline T makeYawAngularErrorContinuous(T yaw, T user_defined_yaw)
    {
        T continuous_error;

        user_defined_yaw = wrapAngleToMinusPiAndPi(user_defined_yaw);

        T raw_error = yaw - user_defined_yaw;

        if (std::abs(raw_error) >= T(M_PI) && std::abs(raw_error) < T(2 * M_PI) && raw_error < T(0))
        {
            continuous_error = std::fmod(raw_error, T(M_PI)) + T(M_PI);
        }
        else if (std::abs(raw_error) >= T(M_PI) && std::abs(raw_error) < T(2 * M_PI) && raw_error > T(0))
        {
            continuous_error = std::fmod(raw_error, T(M_PI)) - T(M_PI);
        }
        else
        {
            continuous_error = std::fmod(raw_error, T(M_PI));
        }

        return continuous_error;
    }

    // Helper function to extract matrices from json files.
    // Example usage : Eigen::Matrix<double, 3, 3> matrix3x3_double = jsonToMatrix<double, 3, 3>(jsonMatrix);
    // Example usage : Eigen::Matrix<float, 4, 4> matrix4x4_float = jsonToMatrix<float, 4, 4>(jsonMatrix);
    template<typename Scalar, int Rows, int Cols>
    inline Eigen::Matrix<Scalar, Rows, Cols> jsonToMatrix(const nlohmann::json& jsonMatrix) {
        Eigen::Matrix<Scalar, Rows, Cols> matrix;
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                matrix(i, j) = jsonMatrix[i][j];
            }
        }
        return matrix;
    }

    // Helper funtion to extract matrices with a scaling factor from json files.
    // Example usage: Eigen::Matrix<double, 3, 3> matrix3x3_double = jsonToScaledMatrix<double, 3, 3>(jsonMatrix);
    // Example usage: Eigen::Matrix<float, 3, 3> matrix3x3_double = jsonToScaledMatrix<float, 3, 3>(jsonMatrix);
    template<typename Scalar, int Rows, int Cols>
    inline Eigen::Matrix<Scalar, Rows, Cols> jsonToScaledMatrix(const nlohmann::json& jsonMatrix) {
        
        // Extract the scaling coefficient
        Scalar scaling_coef = jsonMatrix["scaling_coef"];

        // Iniliatize the Eigen matrix
        Eigen::Matrix<Scalar, Rows, Cols> matrix;

        // Populate the Eigen matrix with the values form the JSON, sclaed by the scaling coefficient
        for (int i = 0; i < Rows; ++i) {
		    for (int j = 0; j < Cols; ++j) {
			    matrix(i, j) = jsonMatrix["matrix"][i][j];
            }
	    }

        matrix *= scaling_coef;

        return matrix;
    }

    // Inline template function to check NaN for scalar values
    template <typename T>
    inline void checkNaN(T value, const char* message) {
        if (std::isnan(value)) {
            throw std::runtime_error(message);
        }
    }

    // Inline template function to check NaN for Eigen matrices
    template <typename Derived>
    inline void checkNaN(const Eigen::MatrixBase<Derived>& matrix, const char* message) {
        if (!matrix.allFinite()) {
            throw std::runtime_error(message);
        }
    }

    // Function to initialize a fixed-size Eigen matrix to zero
    template <typename MatrixType>
    inline void initMat(MatrixType& matrix) {
        matrix.setZero();
    }

    // Function that computes the 321 rotation matrix that transforms a vector from global to local coordinates.
    // rotation_matrix_321_local_to_global = R3 * R2 * R1; // hence
    // rotation_matrix_321_global_to_local = R1_transpose * R2_transpose * R3_transpose;
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 3> rotationMatrix321GlobalToLocal(const Scalar roll, const Scalar pitch, const Scalar yaw) 
    {
        Eigen::Matrix<Scalar, 3, 3> rotation_matrix_321_global_to_local;
        Eigen::Matrix<Scalar, 3, 3> R1_transpose;
        Eigen::Matrix<Scalar, 3, 3> R2_transpose;
        Eigen::Matrix<Scalar, 3, 3> R3_transpose;

        R1_transpose = (Eigen::Matrix<Scalar, 3, 3>() << 
            static_cast<Scalar>(1.0), static_cast<Scalar>(0.0), static_cast<Scalar>(0.0),
            static_cast<Scalar>(0.0),           std::cos(roll),           std::sin(roll),
            static_cast<Scalar>(0.0),          -std::sin(roll),           std::cos(roll)
        ).finished();

        R2_transpose = (Eigen::Matrix<Scalar, 3, 3>() << 
                     std::cos(pitch), static_cast<Scalar>(0.0),         -std::sin(pitch),
            static_cast<Scalar>(0.0), static_cast<Scalar>(1.0), static_cast<Scalar>(0.0),
                     std::sin(pitch), static_cast<Scalar>(0.0),          std::cos(pitch)
        ).finished();

        R3_transpose = (Eigen::Matrix<Scalar, 3, 3>() << 
                       std::cos(yaw),            std::sin(yaw), static_cast<Scalar>(0.0),
                      -std::sin(yaw),            std::cos(yaw), static_cast<Scalar>(0.0),
            static_cast<Scalar>(0.0), static_cast<Scalar>(0.0), static_cast<Scalar>(1.0)
        ).finished();

        rotation_matrix_321_global_to_local = R1_transpose * R2_transpose * R3_transpose;

        return rotation_matrix_321_global_to_local;
    }

    // Function that computes the 321 rotation matrix that transforms a vector from local to global coordinates.
    // rotation_matrix_321_local_to_global = R3 * R2 * R1
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 3> rotationMatrix321LocalToGlobal(const Scalar roll, const Scalar pitch, const Scalar yaw)
    {
        Eigen::Matrix<Scalar, 3, 3> rotation_matrix_321_local_to_global;
        Eigen::Matrix<Scalar, 3, 3> R1;
        Eigen::Matrix<Scalar, 3, 3> R2;
        Eigen::Matrix<Scalar, 3, 3> R3;

        R1 = (Eigen::Matrix<Scalar, 3, 3>() << 
            static_cast<Scalar>(1.0), static_cast<Scalar>(0.0), static_cast<Scalar>(0.0),
            static_cast<Scalar>(0.0),           std::cos(roll),          -std::sin(roll),
            static_cast<Scalar>(0.0),           std::sin(roll),           std::cos(roll)
        ).finished();

        R2 = (Eigen::Matrix<Scalar, 3, 3>() << 
                     std::cos(pitch), static_cast<Scalar>(0.0),          std::sin(pitch),
            static_cast<Scalar>(0.0), static_cast<Scalar>(1.0), static_cast<Scalar>(0.0),
                    -std::sin(pitch), static_cast<Scalar>(0.0),          std::cos(pitch)
        ).finished();

        R3 = (Eigen::Matrix<Scalar, 3, 3>() << 
                       std::cos(yaw),           -std::sin(yaw), static_cast<Scalar>(0.0),
                       std::sin(yaw),            std::cos(yaw), static_cast<Scalar>(0.0),
            static_cast<Scalar>(0.0), static_cast<Scalar>(0.0), static_cast<Scalar>(1.0)
        ).finished();

        rotation_matrix_321_local_to_global  = R3 * R2 * R1;

        return rotation_matrix_321_local_to_global;
    }

    // Function that returns a 3x3 Jacobian matrix inverse given roll and pitch angles.
    // Example usage: double roll = 0.1;
    //                double pitch = 0.2;
    //                Eigen::Matrix3d jacobian_inv = jacobianMatrixInverse(roll, pitch);
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 3> jacobianMatrixInverse(const Scalar roll, const Scalar pitch) {
        // Ensure pitch is not near +/- pi/2 to avoid division by zero
        if (std::abs(std::cos(pitch)) < static_cast<Scalar>(1e-6)) {
            throw std::invalid_argument("Pitch value results in division by zero.");
        }

        Eigen::Matrix<Scalar, 3, 3> jacobian_inverse;

        jacobian_inverse(0, 0) = static_cast<Scalar>(1.0);
        jacobian_inverse(0, 1) = (std::sin(roll) * std::sin(pitch)) / std::cos(pitch);
        jacobian_inverse(0, 2) = (std::cos(roll) * std::sin(pitch)) / std::cos(pitch);

        jacobian_inverse(1, 0) = static_cast<Scalar>(0.0);
        jacobian_inverse(1, 1) = std::cos(roll);
        jacobian_inverse(1, 2) = -std::sin(roll);

        jacobian_inverse(2, 0) = static_cast<Scalar>(0.0);
        jacobian_inverse(2, 1) = std::sin(roll) / std::cos(pitch);
        jacobian_inverse(2, 2) = std::cos(roll) / std::cos(pitch);

        return jacobian_inverse;
    }

    // Function that computes the Jacobian matrix given roll and pitch angles
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 3> jacobianMatrix(const Scalar roll, const Scalar pitch) {

        Eigen::Matrix<Scalar, 3, 3> jacobian_matrix;

        jacobian_matrix(0, 0) = static_cast<Scalar>(1.0);
        jacobian_matrix(0, 1) = static_cast<Scalar>(0.0);
        jacobian_matrix(0, 2) = -(std::sin(pitch));

        jacobian_matrix(1, 0) = static_cast<Scalar>(0.0);
        jacobian_matrix(1, 1) = std::cos(roll);
        jacobian_matrix(1, 2) = std::sin(roll) * std::cos(pitch);

        jacobian_matrix(2, 0) = static_cast<Scalar>(0.0);
        jacobian_matrix(2, 1) = -(std::sin(roll));
        jacobian_matrix(2, 2) = std::cos(roll) * std::cos(pitch);

        return jacobian_matrix;
    }

    // Function that returns a 3x3 Jacobian matrix derivative given roll and pitch angles and their derivatives.
    // Example usage: double roll = 0.1;
    //                double pitch = 0.2;
    //                Eigen::Matrix3d jacobian_dot = jacobianMatrixDerivative(roll, pitch, roll_dot, pitch_dot);
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 3> jacobianMatrixDerivative(const Scalar roll, const Scalar pitch, 
                                                                const Scalar roll_dot, const Scalar pitch_dot) {

        Eigen::Matrix<Scalar, 3, 3> jacobian_matrix_dot;
        
        jacobian_matrix_dot(0, 0) = static_cast<Scalar>(0.0);
        jacobian_matrix_dot(0, 1) = static_cast<Scalar>(0.0);
        jacobian_matrix_dot(0, 2) = -(std::cos(pitch) * pitch_dot);

        jacobian_matrix_dot(1, 0) = static_cast<Scalar>(0.0);
        jacobian_matrix_dot(1, 1) = -(std::sin(roll) * roll_dot);
        jacobian_matrix_dot(1, 2) = (std::cos(roll) * std::cos(pitch) * roll_dot)
                                    - (std::sin(roll) * std::sin(pitch) * pitch_dot);

        jacobian_matrix_dot(2, 0) = static_cast<Scalar>(0.0);
        jacobian_matrix_dot(2, 1) = -(std::cos(roll) * roll_dot);
        jacobian_matrix_dot(2, 2) = -(std::sin(roll) * std::cos(pitch) * roll_dot)
                                    -(std::cos(roll) * std::sin(pitch) * pitch_dot);

        return jacobian_matrix_dot;
    }


    // Function that returns a 3x3 Aerodynamic rotation matrix from Wind to Local frame.
    // Example usage: float alpha
    //                float beta
    //                Eigen::Matrix3f R_W_J = aeroDCMQuadNED(alpha, beta)
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 3> aeroDCMQuadNED(const Scalar alpha, const Scalar beta) {

        Eigen::Matrix<Scalar, 3, 3> DCMQuadNED;

        DCMQuadNED(0, 0) = -(std::cos(beta) * std::sin(alpha));
        DCMQuadNED(0, 1) = (std::sin(alpha) * std::sin(beta));
        DCMQuadNED(0, 2) = -(std::cos(alpha));

        DCMQuadNED(1, 0) = std::sin(beta);
        DCMQuadNED(1, 1) = std::cos(beta);
        DCMQuadNED(1, 2) = static_cast<Scalar>(0.0);

        DCMQuadNED(2, 0) = (std::cos(alpha) * std::cos(beta));
        DCMQuadNED(2, 1) = -(std::cos(alpha) * std::sin(beta));
        DCMQuadNED(2, 2) = -(std::sin(alpha));

        return DCMQuadNED;
    }

    // Function that returns a 3x3 Aerodynamic rotation matrix from Wind to Local frame.
    // Example usage: float alpha
    //                float beta
    //                Eigen::Matrix3f R_W_J = aeroDCMBiplaneNED(alpha, eta)
    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 3, 3> aeroDCMBiplaneNED(const Scalar alpha, const Scalar beta) {

        Eigen::Matrix<Scalar, 3, 3> DCMBiplaneNED;

        DCMBiplaneNED(0,0) = (std::cos(alpha) * std::cos(beta));
        DCMBiplaneNED(0,1) = -(std::cos(alpha) * std::sin(beta));
        DCMBiplaneNED(0,2) = -(std::sin(alpha));

        DCMBiplaneNED(1,0) = std::sin(beta);
        DCMBiplaneNED(1,1) = std::cos(beta);
        DCMBiplaneNED(1,2) = static_cast<Scalar>(0.0);
        
        DCMBiplaneNED(2,0) = (std::sin(alpha) * std::cos(beta));
        DCMBiplaneNED(2,1) = -(std::sin(alpha) * std::sin(beta));
        DCMBiplaneNED(2,2) = std::cos(alpha);
    }

    // Define the state vector as a template
    // Example usage : state_int<double, 78> state_vector_double;
    // Example usage : state_int<float, 100> state_vector_float;
    template<typename T, std::size_t N>
    using rk4_array = boost::array<T, N>;  

    // Assigns elements to dxdt in the model for integration
    template<typename T, int Rows, int Cols, std::size_t N>
    inline void assignElementsToDxdt(Eigen::Matrix<T, Rows, Cols>& entity, boost::array<T, N>& dxdt,
                                     int& current_index)
    {
        std::size_t elements_to_copy = std::min<std::size_t>(entity.size(), dxdt.size() - current_index);
        std::copy(entity.data(), entity.data() + elements_to_copy, dxdt.begin() + current_index);
        current_index += elements_to_copy;
    }   

    // Assign a map for elements from the boost array to the states after integration.
    template<typename T, int Rows, int Cols, std::size_t N>
    inline void assignElementsToMembers(Eigen::Matrix<T, Rows, Cols>& entity, boost::array<T, N>& x,
                                        int& current_index)
    {
        // Map the segment of the boost array to the Eigen matrix
        Eigen::Map<Eigen::Matrix<T, Rows, Cols>>(entity.data(), Rows, Cols) = 
            Eigen::Map<Eigen::Matrix<T, Rows, Cols>>(x.data() + current_index, Rows, Cols);

        // Update the current index by the number of elements in the matrix
        current_index += Rows * Cols;
    }

    // Assign a map for elements from the boost array to the states after integration with the option to reset the 
    // elements 
    template<typename T, int Rows, int Cols, std::size_t N>
    inline void resetElementsToMembers(Eigen::Matrix<T, Rows, Cols>& entity, boost::array<T, N>& x,
                                       int& current_index, boost::array<T, N>& zeros)
    {
        // Map the segment of the boost array to the Eigen matrix
        Eigen::Map<Eigen::Matrix<T, Rows, Cols>>(entity.data(), Rows, Cols) = 
            Eigen::Map<Eigen::Matrix<T, Rows, Cols>>(zeros.data() + current_index, Rows, Cols);

        // If reset_elements is true, set the elements in the boost array to zero
        std::fill(x.data() + current_index, x.data() + current_index + Rows * Cols, static_cast<T>(0));

        // Update the current index by the number of elements in the matrix
        current_index += Rows * Cols;
    }

    // Templated version for fundamental types (restricted to non-Eigen types)
    template<typename T, std::size_t N, typename std::enable_if<std::is_fundamental<T>::value, int>::type = 0>
    inline void assignElementsToDxdt(T& entity, boost::array<T, N>& dxdt, int& current_index)
    {
        dxdt[current_index] = entity;
        current_index++;
    }

    // Templated version for fundamental types (restricted to non-Eigen types)
    template<typename T, std::size_t N, typename std::enable_if<std::is_fundamental<T>::value, int>::type = 0>
    inline void assignElementsToMembers(T& entity, boost::array<T, N>& x, int& current_index)
    {
        entity = x[current_index];
        current_index++;
    }

    // Assign elements to the output array after integration. 
    template<typename T, int Rows, int Cols, std::size_t N>
    inline void assignElementsToX(Eigen::Matrix<T, Rows, Cols>& entity, boost::array<T, N>& X, const int index)    {
        std::size_t elements_to_copy = std::min<std::size_t>(entity.size(), X.size() - index);
        std::copy(entity.data(), entity.data() + elements_to_copy, X.begin() + index);
    }



} // namespace _utilities_

#endif  // HELPER_FUNCTIONS_HPP_
