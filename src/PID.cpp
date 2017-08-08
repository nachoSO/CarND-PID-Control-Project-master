#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    i_error = 0;
    d_error = 0;
    p_error = 0;
}


PID::~PID() {}

/*
Considering a car that drives along a line, how we define the steering angle of a car?
We can steer in proportion to the cross track error (CTE), it means, the larger the error the more
you-re willing to turn towards the target trajectory. In order to implement this we have to use a 
P-controller (proportional controller).
P = tau* CTE

Using only this concept the car could overshoot. Can we solve this?
In order to avoid the overshoot it is used the PD-control. In this case is introduced
the term D (derivative) that acts as the temporal derivative of the CTE. This means that
the car is reducing the error over time. This allows to gracefully approach our target trajectory.
D = CTE(t)-CTE(t-1)

On the other hand, normally, the wheels of a car are not aligned, so,
we can't use the PD controller to solve this problem and it is necessary to correct this deviation.
In order to solve this, is added do so it is necessary to apply the PID control.
You can start steering more the more time goes to compensate this bias introduced.
Now we introduce the Integral to the controller, that is proportional to the integral of the sum of
all the CTE observed, this corrects the robot's motion.
I = tau*sum(CTE)

*/
void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp=Kp;
    this->Ki=Ki;
    this->Kd=Kd;

}

void PID::UpdateError(double cte) {

    i_error += cte;
    d_error = cte - p_error;
    p_error = cte;

}

double PID::TotalError() {
    
    //Ref course: steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte (p,d,i)
    return -Kp * p_error - Kd * d_error - Ki * i_error;

}
