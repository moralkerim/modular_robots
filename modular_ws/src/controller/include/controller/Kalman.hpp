#include <math.h>
#include <stdio.h>
#include "lpf.hpp"

#define LP_FILTER_CUT_FREQ 2*3.14*1
//KALMAN

//#define ROLL_OFFSET   -1.8 //2.0
//#define PITCH_OFFSET  -6.6 //-4.95
#define a 0.1


/*
//COMP
#define ROLL_OFFSET   3.0
#define PITCH_OFFSET -6.05
#define a 0.1 */

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
        float S11_pitch=0, S12_pitch=0, S21_pitch=0, S22_pitch=1e5;
        float S13_pitch, S23_pitch, S31_pitch, S32_pitch, S33_pitch;
        double sa = 1e-3; double sb = 1e-3  ; double sr=7e-2;
        //double sa_p = 5e-1; double sb_p = 1e-1; double sr_p=1e-1;

        //float S11_m_roll, S12_m_roll, S21_m_roll, S22_m_roll;
        float S11_roll=0, S12_roll=0, S21_roll=0, S22_roll=1e5;
        float S13_roll, S23_roll, S31_roll, S32_roll, S33_roll;
     //   float Kt11_roll, Kt21_roll;
/*
        float S11_m_yaw, S12_m_yaw, S21_m_yaw, S22_m_yaw;
        float S11_p_yaw, S12_p_yaw, S21_p_yaw, S22_p_yaw;
        float Kt11_yaw, Kt21_yaw;
*/
        float S11_alt, S12_alt, S21_alt, S22_alt, S13_alt, S23_alt, S31_alt, S32_alt, S33_alt=10000;


        double Qa = 1e5; //0.5 -- onceki deger.
        double Qg = 1e1;

		  float Qb = 1e7;


		  float svel = 2;
		  float sbar = 5;



        const float rad2deg = 180/3.14;

        const int f = 400;
        const double st = 1/(float)f;
        bool gyro_ready;

        float pitch_eski, roll_eski;

    public:
        struct state state;
        float pitch_acc, roll_acc, yaw_acc;
        float roll_gyro, pitch_gyro;
        float pitch_comp, roll_comp;
        float roll_ekf, pitch_ekf;
        float gyro[3], acc[3];
        float Qs = 0.25;
        float salt = 1;
        float acc_vert, alt_gnd, vz, sonar_alt, baro_alt, baro_gnd;

        float PITCH_OFFSET, ROLL_OFFSET;

        //lpf lpf_roll = lpf(0.8544, 0.07282, 0.07282);
        //lpf lpf_pitch = lpf(0.8544, 0.07282, 0.07282);
        lpf lpf_yaw   = lpf(0.8544, 0.07282, 0.07282);

    public:
        Kalman_Filtresi();

        void Run();
        ~Kalman_Filtresi();


};
