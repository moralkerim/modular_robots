#include <math.h>
#include <stdio.h>
#include "LowPassFilter.hpp"

#define LP_FILTER_CUT_FREQ 2*3.14*10


struct state {
    float angles[3];
    float rates[3];
    float bias[3];
};
class Kalman_Filtresi {

    private:
        double roll, pitch, yaw;
        double roll_rate, pitch_rate, yaw_rate;

        float pitch_bias, roll_bias, yaw_bias;

        float S11_m_pitch, S12_m_pitch, S21_m_pitch, S22_m_pitch;
        float S11_p_pitch, S12_p_pitch, S21_p_pitch, S22_p_pitch;
        float Kt11_pitch, Kt21_pitch;
        double sa = 0.01; double sb = 0.01;
        double sa_p = 0.02; double sb_p = 0.02;

        float S11_m_roll, S12_m_roll, S21_m_roll, S22_m_roll;
        float S11_p_roll, S12_p_roll, S21_p_roll, S22_p_roll;
        float Kt11_roll, Kt21_roll;

        float S11_m_yaw, S12_m_yaw, S21_m_yaw, S22_m_yaw;
        float S11_p_yaw, S12_p_yaw, S21_p_yaw, S22_p_yaw;
        float Kt11_yaw, Kt21_yaw;

        double Q = 1; //0.5 -- onceki deger.

        float pitch_acc, roll_acc, yaw_acc;
        const float rad2deg = 180/3.14;

        const int f = 200;
        const double st = 1/(float)f;
    	LowPassFilter lpf;


    public:
        struct state state;


    public:
        Kalman_Filtresi();
        void Run(float gyro[3], float acc[3]);
        ~Kalman_Filtresi();


};
