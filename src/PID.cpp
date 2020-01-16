#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0.0f;
  i_error = 0.0f;
  d_error = 0.0f;
  
  prev_cte = 0.0f;
  total_error = 0.0f;
  
  desired = 0.0f;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  total_error += cte;
  p_error = (cte - desired);
  i_error += cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return p_error * Kp + i_error * Ki + d_error * Kd;  // TODO: Add your total error calc here!
}

double PID::SetDesired(double d) {
  desired = d;
}

double PID::UpdateTaus(double Kp_, double Ki_, double Kd_) 
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}
