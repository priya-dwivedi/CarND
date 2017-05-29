#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0 ;
  double i_error = 0;
  double d_error = 0 ;

  /*
  * Coefficients
  */ 
  double Kp_pid;
  double Ki_pid;
  double Kd_pid;

  bool first_cte = true;

  double prev_cte ;
  double steer;
  double sum_cte;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
};

#endif /* PID_H */
