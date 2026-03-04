#pragma once

/**
 * @brief Simple PID controller with anti-windup
 *
 * u(t) = Kp*e + Ki*∫e dt + Kd*de/dt
 *
 * Anti-windup: integral term is clamped to [-integral_max, integral_max]
 * Output clamp: final output is clamped to [-output_max, output_max]
 */
class PID
{
public:
  PID() = default;

  PID(double kp, double ki, double kd,
      double integral_max = 50.0,
      double output_max   = 10.0)
  : kp_(kp), ki_(ki), kd_(kd),
    integral_max_(integral_max),
    output_max_(output_max)
  {}

  /**
   * @param error     setpoint - measurement
   * @param dt        time step [s]
   * @return control output (effort / torque)
   */
  double compute(double error, double dt)
  {
    if (dt <= 0.0) return 0.0;

    // Integral with anti-windup clamp
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_max_, integral_max_);

    // Derivative (first-order backward difference)
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return std::clamp(output, -output_max_, output_max_);
  }

  void reset()
  {
    integral_   = 0.0;
    prev_error_ = 0.0;
  }

  void set_gains(double kp, double ki, double kd)
  {
    kp_ = kp; ki_ = ki; kd_ = kd;
    reset();
  }

  double kp() const { return kp_; }
  double ki() const { return ki_; }
  double kd() const { return kd_; }

private:
  double kp_ = 0.0, ki_ = 0.0, kd_ = 0.0;
  double integral_     = 0.0;
  double prev_error_   = 0.0;
  double integral_max_ = 50.0;
  double output_max_   = 10.0;
};
