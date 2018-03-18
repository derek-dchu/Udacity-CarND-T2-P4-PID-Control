#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;

    if (fabs(cte) < 0.001) {
        i_error = 0;
    } else {
        i_error += cte;
    }
}

double PID::TotalError() {
    return Kp*p_error + Ki*i_error + Kd*d_error;
}

