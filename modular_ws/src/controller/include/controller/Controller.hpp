
#include <stdio.h>
#include  <vector>
#include "PID.hpp"
#include "lpf.hpp"
#include <math.h>
#include "Modes.h"
#include "state.h"

#define UAV2

#define PWM_UPPER 2000
#define PWM_LOWER 1100
#define ROLL_TRIM 	13
#define PITCH_TRIM  30



class Controller {

    private:
        float pitch_bias, roll_bias, yaw_bias;
        float roll, pitch, yaw;
        float roll_rate, pitch_rate, yaw_rate;

        unsigned int vel_controller_counter;

        float w1, w2, w3, w4; //Motor hizlari
        float pwm_trim = 1550;
        const float rad2deg = 180/3.14;

        const int f = 200;
        float st = 1/(float)f;
        //PID Katsayilari

        const float m = 1.4; //1300 g
        const float g = 9.81;
        const float F_max = 31.23;
        const float F_min = 0;

        //SINGLE UAV

        float Kp_roll = 0.22; //0.3
        float Ki_roll = 0.05;  //0.008
        float Kd_roll = 0.03; //0.007 0.01

        float Kp_pitch = Kp_roll;	//0.8
        float Ki_pitch = Ki_roll;
        float Kd_pitch = Kd_roll;

        float Kp_yaw = 5.0;// 1;
        float Ki_yaw = 9;// 1;

        //TWO UAVS

        /*
        const float Kp_roll = 0.7; //0.3
        const float Ki_roll = 0.05;  //0.008
        const float Kd_roll = 0.02; //0.007 0.01

        const float Kp_pitch = Kp_roll;	//0.8
        const float Ki_pitch = Ki_roll;
        const float Kd_pitch = Kd_roll;

        const float Kp_yaw = 5.0;// 1;
        const float Ki_yaw = 9;// 1;
*/
        const float Kp_angle = 12; //12
        const float Ki_angle = 3;
        const float Kp_alt = 10; //30
        const float Ki_alt = 15;  //3

        const float Kp_velx = -4; //30
        const float Ki_velx = -0;  //3
        const float Kd_velx = 0;

        const float Kp_vely = 4; //30
        const float Ki_vely = 0;  //3
        const float Kd_vely = 0;

        lpf roll_des_filt  = lpf(0.9244, 0.03779, 0.03779);
        lpf pitch_des_filt = lpf(0.9244, 0.03779, 0.03779);
        lpf yaw_des_filt   = lpf(0.9244, 0.03779, 0.03779);


    public:
        Controller();
        void Run (void);
        int controller_output_pwm[4];
        int controller_output_pwm2[4];
        float pd_roll, pd_pitch, p_yaw;
        float roll_rate_des, pitch_rate_des;
        float F;
        struct state state;
        struct state state_des;
        float z_vel, z0,  z,  ch3;
        float vx, x0,  x,  ch2;
        float vy, y0,  y,  ch1;
        Mode mod;
        bool swarm;

        float roll_des, pitch_des, yaw_rate_des;

        PID pid_roll;
        PID pid_pitch;
        PID pid_yaw;
        PID p_alt;
        PID p_velx;
        PID p_vely;
        float alt_thr;
        ~Controller();
};
