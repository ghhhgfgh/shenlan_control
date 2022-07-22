#include "pid_controller.h"

namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki,
                             const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) 
{
  double kp_part = 0.0;
  double ki_part = 0.0;
  double kd_part = 0.0;
  double output = 0.0;

  //1. Proportion
  kp_part = kp_ * error;
  
  //2. Intrgal
  integral_ += error * dt;
  ki_part = ki_ * integral_;

  //3. Derivative:
  if(first_hit_)
  {
    first_hit_ = false;
  }
  else
  {
    kd_part = kd_ * (error - previous_error_) / dt;
  }

  output = kp_part + ki_part + kd_part;
  previous_error_ = error;
  previous_output_ = output;

  return output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() 
{
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true; 
}

}  // namespace control
}  // namespace shenlan
