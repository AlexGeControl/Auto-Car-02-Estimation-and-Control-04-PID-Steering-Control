#ifndef PID_H
#define PID_H

class PID {
public:
  /*
   * Reference:
   */
  double ref;

  /*
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Control:
   */
  double control;

  /*
   * Constructor
   */
  PID(): is_initialized(false) {};

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Initialize PID.
   */
  void Init(double ref, double Kp, double Ki, double Kd, double alpha = 0.95);

  /*
   * Update the PID error variables given actual value.
   */
  void UpdateError(double actual);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

private:
  bool is_initialized;

  double p_error_last;

  double alpha;

  double totalError;
};

#endif /* PID_H */
