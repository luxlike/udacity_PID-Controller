#include "PID.h"
#include <iostream>
#include <math.h>

using std::string;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  p_error = 0;
  i_error = 0;
  d_error = 0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  update_count = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  update_count++;
}

int PID::UpdateCount() {
  return update_count;
}


double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error = -Kp * p_error -Kd * d_error -Ki * i_error;  
  
  if(total_error > 1.0) total_error = 1.0;
  if(total_error < -1.0) total_error = -1.0;
  
  return total_error;  // TODO: Add your total error calc here!
}

void PID::setAccumulateError(double cte_){
  accumulate_error += pow(cte_,2);
}

double PID::getAccumulateError(){
  return accumulate_error;
}

void PID::PrintPIDValue() {
   std::cout << "Kp : " << Kp << " Ki : " << Ki << " Kd : " << Kd << std::endl;
}