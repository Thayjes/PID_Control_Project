#include "PID.h"

using namespace std;
#include <numeric>
#include <iostream>
#include <cmath>
#include <vector>

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    //PID::Kp_t = Kp_t;
    //PID::Ki_t = Ki_t;
    //PID::Kd_t = Kd_t;
    //param_vec.push_back(Kp);
    //param_vec.push_back(Ki);
    //param_vec.push_back(Kd);
    p_error = d_error = i_error = 0.0;
    
    total_error = 0.0;
    best_error = numeric_limits<double>::infinity();
    
    step = 1;
    num_steps_run = 4000;
    num_steps_error = 100;
    
    //dp = {0.2, 0.01, 0.5};
    //dp = {0.1, 0.0001, 0.1, 0.1, 0, 0.02};
    dp = {0.0165104,
        2.44171e-05,
        0.0181615,
        0.081,
        0,
        0.02};
    current_param_index = 0;
    
    added = false;
    subtracted = false;
}

void PID::UpdateError(double cte) {
    // In this function we update the proportional, derivative and integral errors
    // based on the cross track error.
    
    
    // If it is the very first step, to initialize d_error correctly we use the following line
    if(step == 1)
        p_error = cte;
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    
    if(step % (num_steps_run + num_steps_error) > num_steps_error){
        //std::cout << "Step = " << step << ", Num Steps Run = " << num_steps_run << std::endl;
        //std::cout << "Starting to calculate the total error..." << std::endl;
        total_error += pow(cte, 2);
    }
}

void PID::Twiddle(double tol){
    // Here we can implement the twiddle algorithm
    // The twiddle algorithm is as follows:
    // 1. Initialize parameters and run a simulation.
    // 2. Store the total error from this solution and initialize best error.
    // 3. Add a value dp1 to the first parameter. And run the simulation.
    // 4. If total error is better than best error, increment dp1 by small amount.
    // 5. If it is worse than best error, subtract dp1 from the first parameter and run the simulation.
    // 6. After subtracting again check the same.
    // 7. If better, increment dp1 by small amount.
    // 8. If still worse, set parameter1 back to original value and decrement dp1 by small amount.
    // 9. Loop this process for all the parameters.
    // 10. Continue as long as the sum(dp) > tolerance (this may be different for our case).
    if(num_iterations == 0){ // this means we are setting the first best error
        // This means twiddle is being applied for the first time
        // so set the best error to the total error we got from the initial parameters
        best_error = total_error;
        return;
    }
    if((current_param_index % 3) + 3 == 4){
        current_param_index = (current_param_index + 1) % 3 ;
        return;
    }
    current_param_index = (current_param_index % 3) + 3;
    double dpsum = std::accumulate(dp.begin(), dp.end(), 0.0);
    std::cout << "The current dps are = " << std::endl;
    for(double curr_dp : dp){
        std::cout << curr_dp << std::endl;
    }
    std::cout << "The current sum of dp = " << dpsum << std::endl;
    if(true){
        std::cout << "Previous Parameter error = " << total_error << std::endl;
        std::cout << "Best error = " << best_error << std::endl;
        std::cout << "The previously tried parameters are: " << std::endl;
        std::cout << "P = " << Kp << ", I = " << Ki << ", D = " << Kd << std::endl;
        std::cout << "Pthrottle = " << Kp_t << ", Ithrottle = " << Ki_t << ", Dthrottle = " <<
        Kd_t << endl;
        std::cout << "Current param index = " << current_param_index << std::endl;
        if(total_error < best_error){
            // Then this parameter was a success, update the best error
            // increment the change for the parameter and go to the next one
            best_error = total_error;
            dp[current_param_index] *= 1.1;
            current_param_index = (current_param_index + 1) % 3;
            added = subtracted = false;
        }
        // If error was worse, lets try adding to the parameter
        if(!added && !subtracted){
            AddToIndex(current_param_index, dp[current_param_index]);
            added = true;
        }
        else if(added & !subtracted){
            // Adding didnt help the error, lets try subtracting
            AddToIndex(current_param_index, -2*dp[current_param_index]);
            //param_vec[current_param_index] -= 2*dp[current_param_index];
            subtracted = true;
        }
        else{
            // If the error was worse and we have already added and subtracted
            // Revert the parameter to before
            AddToIndex(current_param_index, dp[current_param_index]);
            // Reduce the change for that parameter
            dp[current_param_index] *= 0.9;
            // And go to the next parameter
            current_param_index = (current_param_index + 1) % 3;
            added = subtracted = false;
        }
        // Reset the total error and try running again
        total_error = 0.0;
        std::cout << "The parameters being tried are: " << std::endl;
        std::cout << "P = " << Kp << ", I = " << Ki << ", D = " << Kd << endl;
        std::cout << "Pthrottle = " << Kp_t << ", Ithrottle = " << Ki_t << ", Dthrottle = " <<
        Kd_t << endl;
    }
}

double PID::TotalError() {
    return -Kp*p_error + -Ki*i_error + -Kd*d_error;
}

void PID::AddToIndex(int current_param_index, double current_dp) {
    if(current_param_index == 0){
        Kp = Kp + current_dp;
    }
    if(current_param_index == 1){
        Ki = Ki + current_dp;
    }
    if(current_param_index == 2){
        Kd = Kd + current_dp;
    }
    
    if(current_param_index == 3){
        Kp_t = Kp_t + current_dp;
    }
    if(current_param_index == 4){
        Ki_t = Ki_t + current_dp;
    }
    if(current_param_index == 5){
        Kd_t = Kd_t + current_dp;
    }
    
}

