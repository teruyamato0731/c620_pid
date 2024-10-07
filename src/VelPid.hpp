#ifndef VEL_PID_HPP
#define VEL_PID_HPP
/// @file
/// @brief Provides the VelPid class for velocity PID control.
/// @copyright Copyright (c) 2024 Yoshikawa Teru
/// @license This project is released under the MIT License.
#include <algorithm>
#include <chrono>
#include <cmath>

/// @brief Gains for a PID controller.
struct PidGain {
  float kp;
  float ki;
  float kd;
};

/// @brief Gains and limits for a PID controller.
struct PidParam {
  PidGain gain;
  float min = NAN;
  float max = NAN;
};

/// @brief Velocity Form PID controller.
struct VelPid {
  /// @brief Constructor with the specified gains and limits.
  /// @param param The parameters for the PID controller.
  VelPid(const PidParam& param) : param_{param} {}

  /// @brief Calculates the output of the PID controller.
  float calc(const float target, const float actual, const std::chrono::duration<float>& dt) {
    return calc(target - actual, dt);
  }

  /// @brief Calculates the output of the PID controller.
  float calc(const float error, const std::chrono::duration<float>& dt) {
    const auto sec = dt.count();
    const auto prop = (error - pre_error_) / sec;
    const auto deriv = std::isnan(pre_prop_) ? 0 : (prop - pre_prop_) / sec;
    pre_error_ = error;
    pre_prop_ = prop;
    lpf_deriv_ += (deriv - lpf_deriv_) / 8;
    const auto du = param_.gain.kp * prop + param_.gain.ki * error + param_.gain.kd * lpf_deriv_;
    output_ = std::clamp(output_ + du, param_.min, param_.max);
    return output_;
  }

  /// @brief Resets the PID controller.
  void reset() {
    pre_error_ = 0.0;
    pre_prop_ = NAN;
    lpf_deriv_ = 0.0;
    output_ = 0.0;
  }

  /// @brief Sets the parameters for the PID controller.
  void set_param(const PidParam& param) {
    param_ = param;
    reset();
  }

  /// @brief Sets the gains for the PID controller.
  void set_gain(const PidGain& gain) {
    param_.gain = gain;
    reset();
  }

  /// @brief Sets the limits for the PID controller.
  void set_limit(const float min, const float max) {
    param_.min = min;
    param_.max = max;
  }

 private:
  PidParam param_;
  float pre_error_ = 0.0;
  float pre_prop_ = NAN;
  float lpf_deriv_ = 0.0;
  float output_ = 0.0;
};

#endif  // VEL_PID_HPP
