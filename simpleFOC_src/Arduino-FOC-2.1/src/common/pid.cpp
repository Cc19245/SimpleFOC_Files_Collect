#include "pid.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // output derivative limit [volts/second]
    , limit(limit)         // output supply limit     [volts]
    , integral_prev(0.0)
    , error_prev(0.0)
    , output_prev(0.0)
{
    timestamp_prev = _micros();
}

// PID controller function
float PIDController::operator() (float error){
    // calculate the time from the last call
    unsigned long timestamp_now = _micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations  离散实现
    // proportional part 
    // u_p  = P *e(k)
    float proportional = P * error;
    // Tustin transform of the integral part  双线性变换法，又称塔斯廷s法。用于离散时间转连续时间的系统表示
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I*Ts*0.5*(error + error_prev);
    // antiwindup - limit the output  //抗积分饱和，限制输出
    integral = _constrain(integral, -limit, limit);
    // Discrete derivation  离散微分 
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D*(error - error_prev)/Ts;   

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable  抗积分饱和，限制输出
    output = _constrain(output, -limit, limit);

    // limit the acceleration by ramping the output  通过倾斜输出来限制加速度
    float output_rate = (output - output_prev)/Ts;
    if (output_rate > output_ramp)  //输出速度大于限定值，也就是输出改变的限定值
        output = output_prev + output_ramp*Ts;  //那么输出就用输出限定值
    else if (output_rate < -output_ramp)
        output = output_prev - output_ramp*Ts;
        
    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}

