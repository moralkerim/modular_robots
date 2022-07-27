#include <math.h>
#include <stdio.h>
#include "lpf.hpp"
#include "state.h"


#define LP_FILTER_CUT_FREQ 2*3.14*1
//KALMAN

//#define a 0.1
#define POS_EKF_RATE 4 //100 Hz

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

typedef enum {
	x_axis,
	y_axis
}pos_axis;


class Kalman_Filtresi {

    private:
		float  acc_pos_x_med;
		unsigned int pos_ekf_counter,gps_ekf_counter;
		float Qgps_v=0;
		float pos_st = 0.02;
		float roll, pitch, yaw;
		float roll_rate, pitch_rate, yaw_rate;


        //float S11_m_pitch, S12_m_pitch, S21_m_pitch, S22_m_pitch;
        float S11_pitch=0, S12_pitch=0, S21_pitch=0, S22_pitch;
        float S13_pitch, S23_pitch, S31_pitch, S32_pitch, S33_pitch;
        float sa = 1e-6;  float sr=7e-1;
        //double sa_p = 5e-1; double sb_p = 1e-1; double sr_p=1e-1;

        //float S11_m_roll, S12_m_roll, S21_m_roll, S22_m_roll;
        float S11_roll=0, S12_roll=0, S21_roll=0, S22_roll;
        float S13_roll, S23_roll, S31_roll, S32_roll, S33_roll;
     //   float Kt11_roll, Kt21_roll;

        float S11_yaw=1e5, S12_yaw=0, S21_yaw=0, S22_yaw;
        float S13_yaw, S23_yaw, S31_yaw, S32_yaw, S33_yaw;


        float S11_alt, S12_alt, S21_alt, S22_alt, S13_alt, S23_alt, S31_alt, S32_alt, S33_alt=10000;
        float S11_x, S12_x, S21_x, S22_x ;

        float Sp1_1x, Sp1_2x, Sp1_3x, Sp1_4x, Sp2_1x, Sp2_2x, Sp2_3x, Sp2_4x, Sp3_1x, Sp3_2x, Sp3_3x, Sp3_4x, Sp4_1x, Sp4_2x, Sp4_3x, Sp4_4x=1e9;
        float Sp1_1y, Sp1_2y, Sp1_3y, Sp1_4y, Sp2_1y, Sp2_2y, Sp2_3y, Sp2_4y, Sp3_1y, Sp3_2y, Sp3_3y, Sp3_4y, Sp4_1y, Sp4_2y, Sp4_3y, Sp4_4y=1e9;

        float Qg = 1e1;

        const float Qb = 1e7;



        const float svel = 2;
        const float sbar = 5;

        const float svx  = 50;
        const float spx  = 1;


        const float rad2deg = 180/3.14;

        const int f = 200;
        const double st = 1/(float)f;
        bool gyro_ready;

        float pitch_eski, roll_eski;

    public:
        float acc_pos_x,acc_pos_y;
        struct state state;
        float xbody, ybody;
        bool armed, gps_fixed;
        float pitch_acc, roll_acc, yaw_acc;
        float roll_gyro, pitch_gyro;
        float pitch_comp, roll_comp;
        float roll_ekf, pitch_ekf, yaw_ekf;
        float gyro[3], acc[3];
        float pitch_bias, roll_bias, yaw_bias;
        float sb = 1e-4  ;
        float Qa = 3; //0.5 -- onceki deger.
        float Qay = 3; //0.5 -- onceki deger.
        float x,vx,bax,apx;
        float y,vy,bay,apy;
        float Qap=2e1;
        float Qs = 0.25;
        float Qc = 2.7e-2;
        float Qgps = 2.0;
        float sv = 5e-6;
        float sx = 1e-5;
        float sa_p = 1e0;
        float sb_p = 1e0;

        float salt = 1;
        float acc_vert, alt_gnd, vz, sonar_alt, baro_alt, baro_gnd;
        float accXm, accYm;
        float camx;
        //float xpos, vx;
        float xgps, accx,vgpsx;
        float ygps, accy,vgpsy;

        float GyroXh, GyroYh;
        float xned, yned;
        float v_ground;



        float PITCH_OFFSET=-0.5, ROLL_OFFSET=-4.5;

        //lpf lpf_roll = lpf(0.8544, 0.07282, 0.07282);
        //lpf lpf_pitch = lpf(0.8544, 0.07282, 0.07282);
        lpf lpf_yaw   = lpf(0.8544, 0.07282, 0.07282);
        lpf cam_filt  = lpf(0.9244, 0.03779, 0.03779);

        void EKF_Attitude(euler_angle euler_angle);
        void EKF_Alt(void);
        void EKF_Cam(void);
        void NED2Body(void);
        void EKF_Pos();
        void PredictPos(pos_axis axis);
        void PredictUpdatePos(pos_axis axis);
        void UpdatePos(pos_axis axis);

    public:
        Kalman_Filtresi();

        void Run();
        ~Kalman_Filtresi();


};
