#include "LowPassFilter.hpp"
#define LP_FILTER_CUT_FREQ 1
#define N 10

class PID {

    private:

        float imax=120, imin=-120;
        const int f = 400;
        const float st = 1/(float)f;


        float pd_roll_buf, pd_pitch_buf;
        LowPassFilter lpf;


    public:
        float e_roll, e_pitch, e_eski_roll, e_eski_pitch, ie_roll, ie_pitch; //PID hatalari
        float ie_roll_sat;
        float pd_roll_sat_buf;
        float de, de_filt;
        float de_int;
        double P, I, D, pd;

    public:
        PID();
        double P_Angle(double alpha_des, double alpha, double Kp_angle);
        double PD_Rate(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd);
        double P_Rate_Yaw(double alpha_dot_des, double alpha_dot, double Kp);
        double Sat(double pwm, int max, int min,int thr);
        double Sat(double pwm, int max, int min);
        float pwm2ang(unsigned short int pwm);
        float pwm2rate(unsigned short int pwm);
        float pwm2mot(unsigned short int pwm, int dir);
        double sgn(double v);
        void reset();
        ~PID();
};
