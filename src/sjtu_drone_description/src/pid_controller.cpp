#include "sjtu_drone_description/pid_controller.h"

PIDController::PIDController()
{

}

PIDController::~PIDController()
{

}


void PIDController::Load(sdf::ElementPtr _sdf, const std::string & prefix)
{
  gain_p = 5.0;
  gain_d = 1.0;
  gain_i = 0.0;
  time_constant = 0.0;
  limit = -1.0;

  if (!_sdf) {return;}
  if (_sdf->HasElement(prefix + "ProportionalGain")) {
    gain_p = _sdf->GetElement(prefix + "ProportionalGain")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "DifferentialGain")) {
    gain_d = _sdf->GetElement(prefix + "DifferentialGain")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "IntegralGain")) {
    gain_i = _sdf->GetElement(prefix + "IntegralGain")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "TimeConstant")) {
    time_constant = _sdf->GetElement(prefix + "TimeConstant")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "Limit")) {
    limit = _sdf->GetElement(prefix + "Limit")->Get<double>();
  }
}

double PIDController::update(double new_input, double x, double dx, double dt)
{
  // limit command
  if (limit > 0.0 && fabs(new_input) > limit) {new_input = (new_input < 0 ? -1.0 : 1.0) * limit;}

  // filter command
  if (dt + time_constant > 0.0) {
    input = (dt * new_input + time_constant * input) / (dt + time_constant);
    dinput = (new_input - input) / (dt + time_constant);
  }

  // ==========================================================
  // TODO 1
  //
  // Compute the PID control output for this update step.
  //
  // Requirements:
  // - p : error between the filtered setpoint (input) and the
  //       current measured state (x).
  // - d : error between the filtered setpoint rate (dinput)
  //       and the current measured rate (dx). Note this is
  //       NOT the derivative of p — it compares rates directly.
  // - i : running integral of p, accumulated over time using
  //       the timestep (dt). Must accumulate onto the existing
  //       value of i, not overwrite it.
  // - output : weighted sum of p, d, and i using their
  //       respective gains.
  //
  // Hint:
  // Use:
  //   • input, dinput        (filtered setpoint + setpoint rate)
  //   • x, dx                (current state + current rate)
  //   • dt                   (timestep)
  //   • gain_p, gain_d, gain_i
  //   • p, d, i, output      (member variables to assign/update)
  // ==========================================================

  // YOUR CODE HERE
  p = input - x;
  d = dinput - dx;
  i += p * dt;
  output = gain_p * p + gain_d * d + gain_i * i;

  return output;
}

void PIDController::reset()
{
  input = dinput = 0;
  p = i = d = output = 0;
}