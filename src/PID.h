#ifndef PID_H
#define PID_H
#include<vector>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
    double Kp_t;
    double Ki_t;
    double Kd_t;
  
    
  /*
   * Variables for Twiddle Optimization
   */
    

    // The total error for the current run and the best error of all the runs
    double total_error, best_error;
    
    // Intialize variable for storing the current step number
    int step;
    
    // Initialize variable for determining how many steps to wait until
    // we start calculating the total error.
    int num_steps_run;
    
    // Initialize variable for determining over how many steps we will calculate the error
    int num_steps_error;
    
    // Variable for storing parameter increments/decrements
    std::vector<double> dp;
    
    // Variable for determining which parameter we are currently tuning
    int current_param_index;
    
    // Variable for storing the current number of iterations of parameter updates run
    int num_iterations;
    
    // Variables for informing us if we have already added or subtracted during twiddle
    bool added, subtracted;
    
    // Variable for storing the P,I and D parameters;
    std::vector<double> param_vec;

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
    
  /*
   * Run twiddle on parameters given the tolerance
  */
    void Twiddle(double tol);
    
    void AddToIndex(int current_param_index, double current_dp);
    
};

#endif /* PID_H */
