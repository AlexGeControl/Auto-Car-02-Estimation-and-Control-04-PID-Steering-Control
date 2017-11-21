#include <cmath>
#include "PID.h"

using namespace std;

/*
* PID class.
*/

PID::~PID() {}

/*
 * Initialize PID.
 */
void PID::Init(double ref, double Kp, double Ki, double Kd, double alpha) {
  // Set reference value:
  (*this).ref = ref;
  // Set controller params:
  (*this).Kp = Kp;
  (*this).Ki = Ki;
  (*this).Kd = Kd;

  // Set exponential decay for integral error:
  (*this).alpha = alpha;
  i_error = 0.0;

  totalError = 0.0;
}

/*
 * Update the PID error variables given actual value.
 */
void PID::UpdateError(double actual) {
  // a. Proportional part:
  p_error = ref - actual;
  // Initialize PID state:
  if (!is_initialized) {
    p_error_last = p_error;

    is_initialized = true;
  }
  // b. Differential part:
  d_error = p_error - p_error_last;
  p_error_last = p_error;
  // c. Integral part:
  i_error = alpha * i_error + (1.0 - alpha) * p_error;

  // Generate control:
  control = Kp * p_error + Kd * d_error + Ki * i_error;

  // Update stats:
  totalError += pow(p_error, 2);
}

double PID::TotalError() {
  return totalError;
}
