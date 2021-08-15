#include "Twiddle.h"

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double kp_, double ki_, double kd_) {
  
  is_using = true;
  flag_fw = false;

  num_params = 3;
  best_err = 9999;
  curr_err = 0;
  avg_err = 0;
  idx = 0;

  p = {kp_, ki_, kd_};
  dp = {1.0, 0.01, 1.0};

  best_params = {0, 0, 0};
  best_delta_params = {0, 0, 0};

  is_initialized = false;

  tolerance = 0.0;
}

void Twiddle::setAverageError(double error_) {
  avg_err = error_;
}
  
double Twiddle::getAverageError() {
  return avg_err;
}

void Twiddle::setCurrentError(double error_) {
  curr_err = error_;
}

double Twiddle::getCurrentError() {
  return curr_err;
}

void Twiddle::setBestError(double error_) {
  best_err = error_;
}

double Twiddle::getBestError() {
  return best_err;
}

void Twiddle::setTolerance(double value_) {
  tolerance = value_;
}

double Twiddle::getTolerance() {
  return tolerance;
}

std::vector<double>  Twiddle::UpdateParams() {
  tolerance = dp[0] + dp[1] + dp[2];

  // initialize
  if (!is_initialized) {
    best_err = curr_err;
    flag_fw = true;

    // the first step: update kp gain
    p[idx] += dp[idx];
    is_initialized = true;
  }
  else {
    if (curr_err < best_err) {
      best_err = curr_err;
      best_params = p;
      best_delta_params = dp;
      dp[idx] *= 1.1;
      // switch into the next pid param
      idx = (idx + 1) % 3;
      flag_fw = true;
    }
    else {
      if (flag_fw) {
        p[idx] -= 2 * dp[idx];
        if(p[idx] < 0){
          p[idx] = 0;
        }
        flag_fw = false;
      }
      else {
        p[idx] += dp[idx];
        dp[idx] *= 0.9;
        idx = (idx + 1) % 3;
        flag_fw = true;
      }
    }
    if (flag_fw) {
      p[idx] += dp[idx];
    }
  }

  return p;
}