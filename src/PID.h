#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Update error count.
   * @output The update count
   */
  int UpdateCount();

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /**
   * Accumulate cte error.
   * @param (cte)
   */
  void setAccumulateError(double cte_);

  /**
   * Get accumulate cte error value.
   * @return cte
   */
  double getAccumulateError();

  /**
  * Print Kp, Ki, Kd value
  */ 
  void PrintPIDValue();

  
 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  double accumulate_error; // accumulate error
  int update_count; // pid update count
};

#endif  // PID_H