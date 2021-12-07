#include <stdio.h>
#include  <vector>
#include "PID.hpp"
#define PWM_UPPER 2000
#define PWM_LOWER 1050

class Controller {

    private:
        float pitch_bias, roll_bias, yaw_bias;
        double roll, pitch, yaw;
        double roll_rate, pitch_rate, yaw_rate;

        float w1, w2, w3, w4; //Motor hizlari
        float pwm_trim = 1550;
        const float rad2deg = 180/3.14;

        const int f = 40;
        const float st = 1/(float)f;
        //PID Katsayilari
        double Kp_pitch = 1.5;
        double Ki_pitch = 0.3;
        double Kd_pitch = 0.05*f;

        double Kp_roll = 0.5;
        double Ki_roll = 0.05;
        double Kd_roll = 0.05*f;

        double Kp_yaw = 0.1;

        float Kp_angle = 0.03*f;
        PID pid;

    public:
        Controller();
        std::vector<double> Run (struct state state, float roll_des, float pitch_des, float yaw_rate_des, float gyro[3], float acc[3]);
        ~Controller();
};