#ifndef Twiddle_H
#define Twiddle_H

#include <vector>
#include "PID.h"

class Twiddle {
 public:
  /**
   * Constructor
   */
  Twiddle();

  /**
   * Destructor.
   */
  virtual ~Twiddle();

  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Evaluates error (sum(cte) / count) and runs the twiddle algorithm to adjust initial parameter values.
   * @return vector with updated {Kp, Ki, Kd} values
   */
  std::vector<double>  UpdateParams();  

  void setAverageError(double error_);
  
  double getAverageError();

  void setCurrentError(double error_);
  
  double getCurrentError();

  void setBestError(double error_);
  
  double getBestError();

  void setTolerance(double value_);
  
  double getTolerance();


 private: 
  std::vector<double> p, dp;
  int num_params, idx;
  double best_err, avg_err, curr_err, tolerance;
  bool is_using, flag_fw, is_initialized;
  
  std::vector<double> best_params; 
  std::vector<double> best_delta_params;
};

#endif  // Twiddle_H