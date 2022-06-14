
#include <stdio.h>
#include  <vector>
#include "PID.hpp"
#include "lpf.hpp"
#include <math.h>

#define PWM_UPPER 2000
#define PWM_LOWER 1100
#define ROLL_TRIM 	-1.2
#define PITCH_TRIM -2

class Controller {

    private:
        float pitch_bias, roll_bias, yaw_bias;
        double roll, pitch, yaw;
        double roll_rate, pitch_rate, yaw_rate;

        float w1, w2, w3, w4; //Motor hizlari
        float pwm_trim = 1550;
        const float rad2deg = 180/3.14;

        const int f = 400;
        float st = 1/(float)f;
        //PID Katsayilari

        const float m = 1.4; //1300 g
        const float g = 9.81;
        const float F_max = 31.23;
        const float F_min = 0;
        const double Kp_roll = 0.1; //0.3
        const double Ki_roll = 0.05;  //0.008
        const double Kd_roll = 0.01; //0.007 0.01

        const double Kp_pitch = Kp_roll;	//0.8
        const double Ki_pitch = Ki_roll;
        const double Kd_pitch = Kd_roll;

        const double Kp_yaw = 5.0;// 1;
        const double Ki_yaw = 6.5;// 1;

        const float Kp_angle = 0.03*f;

        const float Kp_alt = 15; //30
        const float Ki_alt = 15;  //3

        lpf roll_des_filt  = lpf(0.9244, 0.03779, 0.03779);
        lpf pitch_des_filt = lpf(0.9244, 0.03779, 0.03779);
        lpf yaw_des_filt   = lpf(0.9244, 0.03779, 0.03779);


    public:
        Controller();
        std::vector<double> Run (struct state state, struct state state_des, int thr);
        std::vector<double> Run (struct state state, struct state state_des, float z_vel, float z0, float z, float ch3);
        int controller_output_pwm[4];
        double pd_roll, pd_pitch, p_yaw;
        float roll_rate_des, pitch_rate_des;
        float F;
        PID pid_roll;
        PID pid_pitch;
        PID pid_yaw;
        PID p_alt;
        float alt_thr;
        ~Controller();
};
