#include <stdio.h>
#include  <vector>
#include "PID.hpp"
#define PWM_UPPER 2000
#define PWM_LOWER 1100
#define ROLL_TRIM 	2.82
#define PITCH_TRIM -16.4

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


        double Kp_roll = 0.13; //0.3
        double Ki_roll = 0.05;  //0.008
        double Kd_roll = 0.02; //0.007 0.01

        double Kp_pitch = Kp_roll;	//0.8
        double Ki_pitch = Ki_roll;
        double Kd_pitch = Kd_roll;

        double Kp_yaw = 9.0;// 1;

        float Kp_angle = 0.03*f;


    public:
        Controller();
        std::vector<double> Run (struct state state, struct state state_des, int thr);
        int controller_output_pwm[4];
        double pd_roll, pd_pitch, p_yaw;
        float roll_rate_des, pitch_rate_des;
        PID pid_roll;
        PID pid_pitch;
        PID pid_yaw;
        ~Controller();
};
