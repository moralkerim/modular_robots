#include "lpf.hpp"
#include <math.h>
#define LP_FILTER_CUT_FREQ 1
#define N 100

class PID {

    private:
		float _e_pos,e_pos;
		float st_pos = 0.05;
        const float imax=120, imin=-120;
        const int f = 200;
        const float st = 1/(float)f;

        const float vz_def = 0.3;
        const float x_inc = vz_def * st;


        float pd_roll_buf, pd_pitch_buf;

        float alpha_dot_des_, alpha_dot_;
        lpf d_filt  = lpf(0.8544, 0.07282, 0.07282);
        float ff_, rate_des_;
        const float K_ff = 1;

    public:
        float e_roll, e_pitch, e_eski_roll, e_eski_pitch, ie_roll, ie_pitch; //PID hatalari
        float ie_roll_rate,ie_roll_sat;
        float pd_roll_sat_buf;
        float de, de_filt;
        float de_int;
        float P, I, D, pd;
        float zi;
        float e_angle;
        float angle0;

    public:
        PID();
        float RateFF(float rate_des);
        float PID_Pos(float pos_des, float pos, float Kp, float Ki, float Kd);
        float P_Angle(float alpha_des, float alpha, float Kp_angle, float Ki_angle);
        float P_Sqrt(float alpha_des, float alpha, float Kp_angle);
        float sqrt_controller(float alpha_des, float _alpha_des, uint8_t angle_counter,float Kff);
        float PD_Rate(float alpha_dot_des, float alpha_dot, float Kp, float Ki, float Kd);
        float PID_Rate2(float alpha_dot_des, float alpha_dot, float alpha, float Kp, float Ki, float Kd, float Kp_angle);
        float P_Rate_Yaw(float alpha_dot_des, float alpha_dot, float Kp);
        float PI_Vel(float z0, float z, float v, float Kp_alt, float Ki_alt, unsigned int ch);
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
