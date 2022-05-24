#include <math.h>
#include <stdio.h>
#include "LowPassFilter.hpp"

#define LP_FILTER_CUT_FREQ 2*3.14*1
#define ROLL_OFFSET   3.0
#define PITCH_OFFSET -4.25
#define a 0.1

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

        //float S11_m_pitch, S12_m_pitch, S21_m_pitch, S22_m_pitch;
        float S11_pitch=0, S12_pitch=0, S21_pitch=0, S22_pitch=0;
        float Kt11_pitch, Kt21_pitch;
        double sa_r = 0.001; double sb_r = 0.001;
        double sa_p = 0.001; double sb_p = 0.001;

        //float S11_m_roll, S12_m_roll, S21_m_roll, S22_m_roll;
        float S11_roll=0, S12_roll=0, S21_roll=0, S22_roll=0;
        float Kt11_roll, Kt21_roll;

        float S11_m_yaw, S12_m_yaw, S21_m_yaw, S22_m_yaw;
        float S11_p_yaw, S12_p_yaw, S21_p_yaw, S22_p_yaw;
        float Kt11_yaw, Kt21_yaw;

        double Q = 10000; //0.5 -- onceki deger.


        const float rad2deg = 180/3.14;

        const int f = 400;
        const double st = 1/(float)f;
        bool gyro_ready;

        float x_prev, y_prev;
        float pitch_eski, roll_eski;

    public:
        struct state state;
        float pitch_acc, roll_acc, yaw_acc;
        float roll_gyro, pitch_gyro;
        float pitch_comp, roll_comp;
        float roll_ekf, pitch_ekf;

    public:
        Kalman_Filtresi();
        float lpf(float x);

        void Run(float gyro[3], float acc[3]);
        ~Kalman_Filtresi();


};
