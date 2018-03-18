#include "twiddle.h"

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::init(std::string in_name, const PID& in_pid, double tol, 
                    const std::vector<double>& dp, int in_n_steps) {
    name = in_name;
    pid = in_pid;
    tolerance = tol;

    // tune in the order of P, D, I
    params = std::vector<double> {pid.Kp, pid.Kd, pid.Ki};
    d_params = dp;
    n_step = in_n_steps;
    curr_n_step = 0;
    curr_i_p = 0;
    c_step = 0;
    is_best_err_initialized = false;
}

PID Twiddle::tune(double cte) {
    if (!is_best_err_initialized) {
        start_time = std::chrono::high_resolution_clock::now();
        initBestError(cte);
        return pid;
    }

    if (std::accumulate(d_params.begin(), d_params.end(), 0.0) < tolerance) {
        is_tuned = true;

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << name << " is tuned in " << c_step << " steps. Elapsed time: " << elapsed.count() << 's';
        std::cout << std::endl;
        std::cout << "Finalized params: ";
        for (auto const& p : params) std::cout << p << ' ';
        std::cout << std::endl;

        return pid;
    }

    if (curr_n_step == n_step) {
        curr_n_step = 0;
        double err = total_eval_err / (n_step / 2);
        total_eval_err = 0.0;

        if (!is_under_2nd_eval) {
            if (err < best_err) {
                best_err = err;
                d_params[curr_i_p] *= 1.1;
            } else {
                params[curr_i_p] -= 2 * d_params[curr_i_p];
                is_under_2nd_eval = true;
            }
        } else {
            is_under_2nd_eval = false;

            if (err < best_err) {
                best_err = err;
                d_params[curr_i_p] *= 1.1;
            } else {
                params[curr_i_p] += d_params[curr_i_p];
                d_params[curr_i_p] *= 0.9;
            }
        }

        if (!is_under_2nd_eval) {
            curr_i_p = (curr_i_p + 1) % params.size();
            params[curr_i_p] += d_params[curr_i_p];
        }
    }

    update(cte);
    return pid;
}

void Twiddle::initBestError(double cte) {
    if (curr_n_step == n_step) {
        curr_n_step = 0;
        best_err = total_eval_err / (n_step / 2);
        total_eval_err = 0.0;
        is_best_err_initialized = true;
        params[curr_i_p] += d_params[curr_i_p];
    }

    update(cte);
}

void Twiddle::update(double cte) {
    pid.Init(params[0], params[2], params[1]);
    pid.UpdateError(cte);

    if (curr_n_step > n_step / 2) {
        total_eval_err += pow(cte, 2);
    }
    
    curr_n_step += 1;
    c_step += 1;

    std::cout << name << " step " << c_step << ": ";
    for (auto const& p : params) std::cout << p << ' ';
    std::cout << std::endl;
}