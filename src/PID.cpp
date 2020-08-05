
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

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  is_cte_init = false;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  if(is_cte_init) d_error = cte - cte_old;
  cte_old = cte;
  is_cte_init = true;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double steering;
  steering = -Kp * p_error - Ki * i_error - Kd * d_error; 
  if(steering < -1) steering = -1;
  if(steering > 1) steering = 1; 
  return steering;// TODO: Add your total error calc here!
}