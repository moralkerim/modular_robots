#include "lpf.hpp"
#include <math.h>
#define LP_FILTER_CUT_FREQ 1
#define N 100

class PID {

    private:

        const float imax=120, imin=-120;
        const int f = 400;
        const float st = 1/(float)f;

        const float vz_def = 0.3;
        const float x_inc = vz_def * st;


        float pd_roll_buf, pd_pitch_buf;

        float alpha_dot_des_;
        lpf d_filt  = lpf(0.9512, 0.02439, 0.02439);

    public:
        float e_roll, e_pitch, e_eski_roll, e_eski_pitch, ie_roll, ie_pitch; //PID hatalari
        float ie_roll_sat;
        float pd_roll_sat_buf;
        float de, de_filt;
        float de_int;
        float P, I, D, pd;
        float zi;
        float e_angle;
        float angle0;

    public:
        PID();
        float P_Angle(float alpha_des, float alpha, float Kp_angle);
        float P_Sqrt(float alpha_des, float alpha, float Kp_angle);
        float PD_Rate(float alpha_dot_des, float alpha_dot, float Kp, float Ki, float Kd);
        float PID_Rate2(float alpha_dot_des, float alpha_dot, float Kp, float Ki, float Kd);
        float P_Rate_Yaw(float alpha_dot_des, float alpha_dot, float Kp);
        float PI_Alt(float z0, float z, float v, float Kp_alt, float Ki_alt, unsigned int ch3);
        float Sat(float pwm, int max, int min,int thr);
        float Sat(float pwm, int max, int min);
        float pwm2ang(unsigned short int pwm);
        float pwm2rate(unsigned short int pwm);
        float pwm2mot(unsigned short int pwm, int dir);
        unsigned int F2thr(float F);
        uint8_t sgn(float v);
        void reset();
        ~PID();
};
