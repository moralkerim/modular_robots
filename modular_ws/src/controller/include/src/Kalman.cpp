#include "Kalman.hpp"
#include <math.h>


Kalman_Filtresi::Kalman_Filtresi() {};

void Kalman_Filtresi::Run(float gyro[3], float acc[3]) {
  float accX = acc[0]; 
  float accY = acc[1];
  float accZ = acc[2];

  float gyroX = gyro[0]; 
  float gyroY = gyro[1];
  float gyroZ = gyro[2];

    //---IMU KİSMİ----
    //=================================
  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);

  pitch_acc =  asin(accX/acctop)*rad2deg;
  roll_acc  =  asin(accY/acctop)*rad2deg;
  yaw_acc   =  asin(accZ/acctop)*rad2deg;

  #if USE_SIM
    attitude.roll_acc = roll_acc;
    attitude.pitch_acc = pitch_acc;
    attitude.yaw_acc = yaw_acc;
    //printf("\npitc_acc: %.2f", pitch_acc);
  #endif
    
  //Pitch angle
	//**Tahmin**
	pitch = pitch - pitch_bias*st + gyroY*st;
	S11_m_pitch = 2*sa+st*st*sb; S12_m_pitch=-st*sb;
	S21_m_pitch = -st*sb; 	   S22_m_pitch=2*sb;

	//**Düzeltme**
	Kt11_pitch = S11_m_pitch / (S11_m_pitch+Q);
	Kt21_pitch = S21_m_pitch / (S21_m_pitch+Q);

	pitch = pitch - Kt11_pitch*(pitch-pitch_acc);
	pitch_bias = pitch_bias - Kt21_pitch*(pitch-pitch_acc);

	S11_p_pitch = -S11_m_pitch*(Kt11_pitch-1);  S12_p_pitch = -S12_m_pitch*(Kt11_pitch-1);
	S21_p_pitch = S21_m_pitch-S11_m_pitch*Kt21_pitch; S22_p_pitch = S22_m_pitch-S12_m_pitch*Kt21_pitch;

	S11_m_pitch = S11_p_pitch; S12_m_pitch = S12_p_pitch; S21_m_pitch = S21_p_pitch; S22_m_pitch = S22_p_pitch; 

  pitch_rate = gyroY;
    //=================================

  //Roll angle
	//**Tahmin**
	roll = roll - roll_bias*st + gyroX*st;
	S11_m_roll = 2*sa+st*st*sb; S12_m_roll=-st*sb;
	S21_m_roll = -st*sb; 	   S22_m_roll=2*sb;

	//**Düzeltme**
	Kt11_roll = S11_m_roll / (S11_m_roll+Q);
	Kt21_roll = S21_m_roll / (S21_m_roll+Q);

	roll = roll - Kt11_roll*(roll-roll_acc);
	roll_bias = roll_bias - Kt21_roll*(roll-roll_acc);

	S11_p_roll = -S11_m_roll*(Kt11_roll-1);  S12_p_roll = -S12_m_roll*(Kt11_roll-1);
	S21_p_roll = S21_m_roll-S11_m_roll*Kt21_roll; S22_p_roll = S22_m_roll-S12_m_roll*Kt21_roll;

	S11_m_roll = S11_p_roll; S12_m_roll = S12_p_roll; S21_m_roll = S21_p_roll; S22_m_roll = S22_p_roll; 
  roll_rate = gyroX;
    //=================================

   //Yaw angle
	//**Tahmin**
	yaw = yaw - yaw_bias*st + gyroZ*st;
	S11_m_yaw = 2*sa+st*st*sb; S12_m_yaw=-st*sb;
	S21_m_yaw = -st*sb; 	   S22_m_yaw=2*sb;

	//**Düzeltme**
	Kt11_yaw = S11_m_yaw / (S11_m_yaw+Q);
	Kt21_yaw = S21_m_yaw / (S21_m_yaw+Q);

	yaw = yaw - Kt11_yaw*(yaw-yaw_acc);
	yaw_bias = yaw_bias - Kt21_yaw*(yaw-yaw_acc);

	S11_p_yaw = -S11_m_yaw*(Kt11_yaw-1);  S12_p_yaw = -S12_m_yaw*(Kt11_yaw-1);
	S21_p_yaw = S21_m_yaw-S11_m_yaw*Kt21_yaw; S22_p_yaw = S22_m_yaw-S12_m_yaw*Kt21_yaw;

	S11_m_yaw = S11_p_yaw; S12_m_yaw = S12_p_yaw; S21_m_yaw = S21_p_yaw; S22_m_yaw = S22_p_yaw; 
  yaw_rate = gyroZ;
    //=================================

    state.angles[0] = roll;
    state.angles[1] = pitch;
    state.angles[2] = yaw;

    state.rates[0] = roll_rate;
    state.rates[1] = pitch_rate;
    state.rates[2] = yaw_rate;

    state.bias[0] = roll_bias;
    state.bias[1] = pitch_bias;
    state.bias[2] = yaw_bias;

}

Kalman_Filtresi::~Kalman_Filtresi() {};