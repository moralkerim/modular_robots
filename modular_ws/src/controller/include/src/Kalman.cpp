#include "Kalman.hpp"
#include <math.h>


Kalman_Filtresi::Kalman_Filtresi()  {

}


void Kalman_Filtresi::Run() {

  float accX = acc[0]; 
  float accY = acc[1];
  float accZ = acc[2];

  float gyroX = gyro[0]; 
  float gyroY = gyro[1];
  float gyroZ = gyro[2];

    //---IMU KİSMİ----
    //=================================
  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);

  pitch_acc =  asin(accX/acctop)*rad2deg + PITCH_OFFSET;
  roll_acc  =  asin(accY/acctop)*rad2deg + ROLL_OFFSET;

  //roll_acc-=-0.67;	pitch_acc-=-1.15;	//İvmeölçer ile okunan hata değerleri offsetlemesi


  //pitch_acc = lpf(pitch_acc);
  //roll_acc = lpf(roll_acc);


  pitch_gyro = gyroY * st;
  roll_gyro =  gyroX * st;


  //yaw_acc   =  asin(accZ/acctop)*rad2deg;

  /*
  pitch_acc = lpf.update(pitch_acc);
  roll_acc = lpf.update(roll_acc);
  */
  //yaw_acc = lpf.update(yaw_acc);

  #if USE_SIM
    attitude.roll_acc = roll_acc;
    attitude.pitch_acc = pitch_acc;
    attitude.yaw_acc = yaw_acc;
    //printf("\npitc_acc: %.2f", pitch_acc);
  #endif
    
    //if(gyro_ready) {


    pitch_comp=(pitch_gyro+pitch_eski)*0.998+pitch_acc*0.002;	//Tümleyen filtre
    roll_comp =(roll_gyro+roll_eski)*0.998+roll_acc*0.002;		//Tümleyen filtre


    //ANGLE PREDICTION
    roll_ekf = (roll_ekf) + st*(roll_rate);

    S11_roll = S11_roll + sa + S31_roll*st + (st*st*(S13_roll + S33_roll*st))/st;
    S12_roll = S12_roll + S32_roll*st;
    S13_roll = S13_roll + S33_roll*st;

    S21_roll = S21_roll + S23_roll*(st);
    S22_roll = S22_roll + sb;
    //S23_roll = S23_roll;

    S31_roll = S31_roll + S33_roll*(st);
    //S32_roll = S32_roll;
    S33_roll = S33_roll + sr;

    //ANGLE CORRECTION
    float A = (Qa*Qg + Qa*S22_roll + Qa*S23_roll + Qa*S32_roll + Qa*S33_roll + Qg*S11_roll + S11_roll*S22_roll - S12_roll*S21_roll + S11_roll*S23_roll - S13_roll*S21_roll + S11_roll*S32_roll - S12_roll*S31_roll + S11_roll*S33_roll - S13_roll*S31_roll);
    float Kt11_att = 1 - (Qa*(Qg + S22_roll + S23_roll + S32_roll + S33_roll))/A;
    float Kt12_att = (Qa*(S12_roll + S13_roll))/A;
    float Kt21_att = (Qg*S21_roll + S21_roll*S32_roll - S22_roll*S31_roll + S21_roll*S33_roll - S23_roll*S31_roll)/A;
    float Kt22_att = (Qa*S22_roll + Qa*S23_roll + S11_roll*S22_roll - S12_roll*S21_roll + S11_roll*S23_roll - S13_roll*S21_roll)/A;
    float Kt31_att = (Qg*S31_roll - S21_roll*S32_roll + S22_roll*S31_roll - S21_roll*S33_roll + S23_roll*S31_roll)/A;
    float Kt32_att = (Qa*S32_roll + Qa*S33_roll + S11_roll*S32_roll - S12_roll*S31_roll + S11_roll*S33_roll - S13_roll*S31_roll)/A;

    roll_ekf = (roll_ekf) + Kt11_att*((roll_acc) - (roll_ekf)) - Kt12_att*((roll_bias) - (gyroX) + (roll_rate));


    roll_bias = (roll_bias) + Kt21_att*((roll_acc) - (roll_ekf)) - Kt22_att*((roll_bias) - (gyroX) + (roll_rate));


    roll_rate = (roll_rate) + Kt31_att*((roll_acc) - (roll_ekf)) - Kt32_att*((roll_bias) - (gyroX) + (roll_rate));


    S11_roll = - S11_roll*(Kt11_att - 1) - Kt12_att*S21_roll - Kt12_att*S31_roll;


    S12_roll = - S12_roll*(Kt11_att - 1) - Kt12_att*S22_roll - Kt12_att*S32_roll;


    S13_roll = - S13_roll*(Kt11_att - 1) - Kt12_att*S23_roll - Kt12_att*S33_roll;


    S21_roll = - S21_roll*(Kt22_att - 1) - Kt21_att*S11_roll - Kt22_att*S31_roll;


    S22_roll = - S22_roll*(Kt22_att - 1) - Kt21_att*S12_roll - Kt22_att*S32_roll;


    S23_roll = - S23_roll*(Kt22_att - 1) - Kt21_att*S13_roll - Kt22_att*S33_roll;


    S31_roll = - S31_roll*(Kt32_att - 1) - Kt31_att*S11_roll - Kt32_att*S21_roll;


    S32_roll = - S32_roll*(Kt32_att - 1) - Kt31_att*S12_roll - Kt32_att*S22_roll;


    S33_roll = - S33_roll*(Kt32_att - 1) - Kt31_att*S13_roll - Kt32_att*S23_roll;

    //============================

    //ANGLE PREDICTION
    pitch_ekf = (pitch_ekf) + st*(pitch_rate);

    S11_pitch = S11_pitch + sa + S31_pitch*st + (st*st*(S13_pitch + S33_pitch*st))/st;
    S12_pitch = S12_pitch + S32_pitch*st;
    S13_pitch = S13_pitch + S33_pitch*st;

    S21_pitch = S21_pitch + S23_pitch*(st);
    S22_pitch = S22_pitch + sb;
    //S23_pitch = S23_pitch;

    S31_pitch = S31_pitch + S33_pitch*(st);
    //S32_pitch = S32_pitch;
    S33_pitch = S33_pitch + sr;

    //ANGLE CORRECTION
    A = (Qa*Qg + Qa*S22_pitch + Qa*S23_pitch + Qa*S32_pitch + Qa*S33_pitch + Qg*S11_pitch + S11_pitch*S22_pitch - S12_pitch*S21_pitch + S11_pitch*S23_pitch - S13_pitch*S21_pitch + S11_pitch*S32_pitch - S12_pitch*S31_pitch + S11_pitch*S33_pitch - S13_pitch*S31_pitch);
     Kt11_att = 1 - (Qa*(Qg + S22_pitch + S23_pitch + S32_pitch + S33_pitch))/A;
     Kt12_att = (Qa*(S12_pitch + S13_pitch))/A;
     Kt21_att = (Qg*S21_pitch + S21_pitch*S32_pitch - S22_pitch*S31_pitch + S21_pitch*S33_pitch - S23_pitch*S31_pitch)/A;
     Kt22_att = (Qa*S22_pitch + Qa*S23_pitch + S11_pitch*S22_pitch - S12_pitch*S21_pitch + S11_pitch*S23_pitch - S13_pitch*S21_pitch)/A;
     Kt31_att = (Qg*S31_pitch - S21_pitch*S32_pitch + S22_pitch*S31_pitch - S21_pitch*S33_pitch + S23_pitch*S31_pitch)/A;
     Kt32_att = (Qa*S32_pitch + Qa*S33_pitch + S11_pitch*S32_pitch - S12_pitch*S31_pitch + S11_pitch*S33_pitch - S13_pitch*S31_pitch)/A;

    pitch_ekf = (pitch_ekf) + Kt11_att*((pitch_acc) - (pitch_ekf)) - Kt12_att*((pitch_bias) - (gyroY) + (pitch_rate));

    //pitch_ekf = 0;
    pitch_bias = (pitch_bias) + Kt21_att*((pitch_acc) - (pitch_ekf)) - Kt22_att*((pitch_bias) - (gyroY) + (pitch_rate));
    //pitch_bias = 0;

    pitch_rate = (pitch_rate) + Kt31_att*((pitch_acc) - (pitch_ekf)) - Kt32_att*((pitch_bias) - (gyroY) + (pitch_rate));


    S11_pitch = - S11_pitch*(Kt11_att - 1) - Kt12_att*S21_pitch - Kt12_att*S31_pitch;


    S12_pitch = - S12_pitch*(Kt11_att - 1) - Kt12_att*S22_pitch - Kt12_att*S32_pitch;


    S13_pitch = - S13_pitch*(Kt11_att - 1) - Kt12_att*S23_pitch - Kt12_att*S33_pitch;


    S21_pitch = - S21_pitch*(Kt22_att - 1) - Kt21_att*S11_pitch - Kt22_att*S31_pitch;


    S22_pitch = - S22_pitch*(Kt22_att - 1) - Kt21_att*S12_pitch - Kt22_att*S32_pitch;


    S23_pitch = - S23_pitch*(Kt22_att - 1) - Kt21_att*S13_pitch - Kt22_att*S33_pitch;


    S31_pitch = - S31_pitch*(Kt32_att - 1) - Kt31_att*S11_pitch - Kt32_att*S21_pitch;


    S32_pitch = - S32_pitch*(Kt32_att - 1) - Kt31_att*S12_pitch - Kt32_att*S22_pitch;


    S33_pitch = - S33_pitch*(Kt32_att - 1) - Kt31_att*S13_pitch - Kt32_att*S23_pitch;

    /*
    pitch_ekf = pitch_ekf - st*pitch_bias + gyroY*(st) + ((pitch_acc - pitch_ekf + st*pitch_bias - gyroY*(st))*(S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st)))/(Q + S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st));
  pitch_bias = pitch_bias + ((S21_pitch + (sb_p) - S22_pitch*(st))*(pitch_acc - pitch_ekf + st*pitch_bias - gyroY*(st)))/(Q + S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st));

  S11_pitch = -((S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st))/(Q + S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st)) - 1)*(S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st));
  S12_pitch = -((S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st))/(Q + S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st)) - 1)*(S12_pitch + (sa_p) - S22_pitch*st);
  S21_pitch = S21_pitch + (sb_p) - S22_pitch*(st) - ((S21_pitch + (sb_p) - S22_pitch*(st))*(S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st)))/(Q + S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st));
  S22_pitch = S22_pitch + (sb_p) - ((S21_pitch + (sb_p) - S22_pitch*(st))*(S12_pitch + (sa_p) - S22_pitch*st))/(Q + S11_pitch + (sa_p) - S21_pitch*st - (st)*(S12_pitch - S22_pitch*st));
    pitch_rate = gyroY;

    roll_ekf = roll_ekf - st*roll_bias + gyroX*(st) + ((roll_acc - roll_ekf + st*roll_bias - gyroX*(st))*(S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st)))/(Q + S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st));
  roll_bias = roll_bias + ((S21_roll + (sb_r) - S22_roll*(st))*(roll_acc - roll_ekf + st*roll_bias - gyroX*(st)))/(Q + S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st));

  S11_roll = -((S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st))/(Q + S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st)) - 1)*(S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st));
  S12_roll = -((S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st))/(Q + S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st)) - 1)*(S12_roll + (sa_r) - S22_roll*st);
  S21_roll = S21_roll + (sb_r) - S22_roll*(st) - ((S21_roll + (sb_r) - S22_roll*(st))*(S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st)))/(Q + S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st));
  S22_roll = S22_roll + (sb_r) - ((S21_roll + (sb_r) - S22_roll*(st))*(S12_roll + (sa_r) - S22_roll*st))/(Q + S11_roll + (sa_r) - S21_roll*st - (st)*(S12_roll - S22_roll*st));
    roll_rate = gyroX; */

/*
  roll_rate  = lpf_roll.Run(gyroX);
  pitch_rate = lpf_pitch.Run(gyroY);*/

  yaw_rate   = lpf_yaw.Run(gyroZ);

    //=================================

    //}

    /*
    else {
    	roll_ekf = roll_acc;
    	pitch_ekf = pitch_acc;

    	roll_comp  = roll_acc;
    	pitch_comp = pitch_acc;

    	gyro_ready = true;
    } */

      float u = acc_vert;

      //ALT PREDICTION
	  alt_gnd = (alt_gnd) + st*(vz) + (u*(st)*st)/2;
	  vz = (vz) + u*(st);
	  //baro_gnd = (baro_gnd);

	  S11_alt = S11_alt + salt + S21_alt*st + (st)*(S12_alt + S22_alt*st);
	  S12_alt = S12_alt + S22_alt*st;
	  S13_alt = S13_alt + S23_alt*st;

	  S21_alt = S21_alt + S22_alt*(st);
	  S22_alt =  S22_alt + svel;
	  //S23_alt = S23_alt;

	  S31_alt = S31_alt + S32_alt*(st);
	  //S32_alt = S32_alt;
	  S33_alt = S33_alt + sbar;

	  A = (Qb*Qs + Qb*S11_alt + Qs*S11_alt + Qs*S13_alt + Qs*S31_alt + Qs*S33_alt + S11_alt*S33_alt - S13_alt*S31_alt);

	  //ALT CORRECTION
	  float Kt11 = (Qs*(S11_alt + S13_alt))/A;
	  float Kt12 = (Qb*S11_alt + S11_alt*S33_alt - S13_alt*S31_alt)/A;

	  float Kt21 = (Qs*S21_alt + Qs*S23_alt + S11_alt*S23_alt - S13_alt*S21_alt)/A;
	  float Kt22 = (Qb*S21_alt - S11_alt*S23_alt + S13_alt*S21_alt + S21_alt*S33_alt - S23_alt*S31_alt)/A;

	  float Kt31 = (Qs*S31_alt + Qs*S33_alt + S11_alt*S33_alt - S13_alt*S31_alt)/A;
	  float Kt32 = (Qb*S31_alt - S11_alt*S33_alt + S13_alt*S31_alt)/A;

	  alt_gnd = (alt_gnd) - Kt12*((alt_gnd) - (sonar_alt)) - Kt11*((alt_gnd) - (baro_alt) + (baro_gnd));


	  vz = (vz) - Kt22*((alt_gnd) - (sonar_alt)) - Kt21*((alt_gnd) - (baro_alt) + (baro_gnd));


	  baro_gnd = (baro_gnd) - Kt32*((alt_gnd) - (sonar_alt)) - Kt31*((alt_gnd) - (baro_alt) + (baro_gnd));


	  S11_alt =  - S11_alt*(Kt11 + Kt12 - 1) - Kt11*S31_alt;


	  S12_alt = - S12_alt*(Kt11 + Kt12 - 1) - Kt11*S32_alt;


	  S13_alt = - S13_alt*(Kt11 + Kt12 - 1) - Kt11*S33_alt;


	  S21_alt =  S21_alt - S11_alt*(Kt21 + Kt22) - Kt21*S31_alt;


	  S22_alt =  S22_alt - S12_alt*(Kt21 + Kt22) - Kt21*S32_alt;


	  S23_alt =  S23_alt - S13_alt*(Kt21 + Kt22) - Kt21*S33_alt;


	  S31_alt = - S31_alt*(Kt31 - 1) - S11_alt*(Kt31 + Kt32);


	  S32_alt = - S32_alt*(Kt31 - 1) - S12_alt*(Kt31 + Kt32);


	  S33_alt = - S33_alt*(Kt31 - 1) - S13_alt*(Kt31 + Kt32);


	pitch_eski=pitch_comp;
	roll_eski=roll_comp;

    state.angles[0] = roll_ekf;
    state.angles[1] = pitch_ekf;
    state.angles[2] = 0;

    state.rates[0] = roll_rate;
    state.rates[1] = pitch_rate;
    state.rates[2] = yaw_rate;

    state.bias[0] = roll_bias;
    state.bias[1] = pitch_bias;
    state.bias[2] = yaw_bias;

}

Kalman_Filtresi::~Kalman_Filtresi() {}
