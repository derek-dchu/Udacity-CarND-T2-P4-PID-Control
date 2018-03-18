#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <iostream>
#include <vector>
#include <numeric>
#include <math.h>
#include <chrono>
#include "PID.h"


class Twiddle {
public:
    
    std::string name;
    bool is_tuned;

    // count of total steps
    signed int c_step;

    /*
    * Constructor
    */
    Twiddle();

    /*
    * Destructor.
    */
    virtual ~Twiddle();

    void init(std::string name, const PID& pid, double tolerance, const std::vector<double>& d_params, int n_step);
 
    /**
     * Tune hyperparameters
     */
    PID tune(double cte);

private:

    PID pid;
    double tolerance;
    std::vector<double> params;

    // step length for each hyperparameter
    std::vector<double> d_params;

    // number of steps required to evaluate new error
    int n_step;

    // current number of steps in evaluation
    int curr_n_step;

    // current ith hyperparameter under tuning
    int curr_i_p;

    double total_eval_err;
    double best_err;
    bool is_best_err_initialized;
    bool is_under_2nd_eval;

    // twiddle start time
    std::chrono::high_resolution_clock::time_point start_time;
    
    /**
     * Initalize best error
     */
    void initBestError(double cte);

    /**
     * Update hyperparameters
     */
    void update(double cte);
};

#endif /* TWIDDLE_H */