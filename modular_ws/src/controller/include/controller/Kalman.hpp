#include <math.h>
#include <stdio.h>
#include "lpf.hpp"
#include "state.h"


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

typedef enum {
	ROLL,
	PITCH,
	YAW
}euler_angle;

class Kalman_Filtresi {

    private:
		float roll, pitch, yaw;
		float roll_rate, pitch_rate, yaw_rate;


        //float S11_m_pitch, S12_m_pitch, S21_m_pitch, S22_m_pitch;
        float S11_pitch=0, S12_pitch=0, S21_pitch=0, S22_pitch;
        float S13_pitch, S23_pitch, S31_pitch, S32_pitch, S33_pitch;
        float sa = 1e-6;  float sr=7e-2;
        //double sa_p = 5e-1; double sb_p = 1e-1; double sr_p=1e-1;

        //float S11_m_roll, S12_m_roll, S21_m_roll, S22_m_roll;
        float S11_roll=0, S12_roll=0, S21_roll=0, S22_roll;
        float S13_roll, S23_roll, S31_roll, S32_roll, S33_roll;
     //   float Kt11_roll, Kt21_roll;

        float S11_yaw=1e5, S12_yaw=0, S21_yaw=0, S22_yaw;
        float S13_yaw, S23_yaw, S31_yaw, S32_yaw, S33_yaw;


        float S11_alt, S12_alt, S21_alt, S22_alt, S13_alt, S23_alt, S31_alt, S32_alt, S33_alt=10000;
        float S11_x, S12_x, S21_x, S22_x ;

        float Qg = 1e1;

        const float Qb = 1e7;



        const float svel = 2;
        const float sbar = 5;

        const float svx  = 50;
        const float spx  = 1;


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
        float roll_ekf, pitch_ekf, yaw_ekf;
        float gyro[3], acc[3];
        float pitch_bias, roll_bias, yaw_bias;
        float sb = 1e-5  ;
        float Qa = 5e4; //0.5 -- onceki deger.

        float Qs = 0.25;
        float Qc = 2.7e-2;

        float salt = 1;
        float acc_vert, alt_gnd, vz, sonar_alt, baro_alt, baro_gnd;
        float accXm, accYm;
        float camx;
        float xpos, vx;
        float GyroXh, GyroYh;



        float PITCH_OFFSET=-2.98847938, ROLL_OFFSET=2.82364707;

        //lpf lpf_roll = lpf(0.8544, 0.07282, 0.07282);
        //lpf lpf_pitch = lpf(0.8544, 0.07282, 0.07282);
        lpf lpf_yaw   = lpf(0.8544, 0.07282, 0.07282);
        lpf cam_filt  = lpf(0.9244, 0.03779, 0.03779);

        void EKF_Attitude(euler_angle euler_angle);
        void EKF_Alt(void);
        void EKF_Cam(void);

    public:
        Kalman_Filtresi();

        void Run();
        ~Kalman_Filtresi();


};
