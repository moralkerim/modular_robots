#include "Kalman.hpp"
#include <math.h>


Kalman_Filtresi::Kalman_Filtresi()  {

}

void Kalman_Filtresi::EKF_Attitude(euler_angle euler_angle) {
	  float accX = acc[0];
	  float accY = acc[1];
	  float accZ = acc[2];

	  float gyroX = gyro[0];
	  float gyroY = gyro[1];
	  float gyroZ = gyro[2];

	  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);

	  pitch_acc =  asin(accX/acctop)*rad2deg + PITCH_OFFSET;
	  roll_acc  =  asin(accY/acctop)*rad2deg + ROLL_OFFSET;

	float angle_ekf, angle_rate, angle_bias, angle_acc, gyro;
	float S11_angle, S12_angle, S13_angle, S21_angle, S22_angle, S23_angle, S31_angle, S32_angle, S33_angle ;
    //ANGLE PREDICTION
	switch(euler_angle) {
		case ROLL:
			angle_ekf = roll_ekf;
			angle_rate = roll_rate;
			angle_bias = roll_bias;
			angle_acc = roll_acc;
			gyro = gyroX;
			sa = 1e-6;
			Qa = 5e4;
			Qg = 1e1;

			S11_angle = S11_roll;
			S12_angle = S12_roll;
			S13_angle = S13_roll;
			S21_angle = S21_roll;
			S22_angle = S22_roll;
			S23_angle = S23_roll;
			S31_angle = S31_roll;
			S32_angle = S32_roll;
			S33_angle = S33_roll;
			break;

		case PITCH:
			angle_ekf = pitch_ekf;
			angle_rate = pitch_rate;
			angle_bias = pitch_bias;
			angle_acc = pitch_acc;
			gyro = gyroY;
			sa = 1e-6;
			Qa = 5e4;
			Qg = 1e1;

			S11_angle = S11_pitch;
			S12_angle = S12_pitch;
			S13_angle = S13_pitch;
			S21_angle = S21_pitch;
			S22_angle = S22_pitch;
			S23_angle = S23_pitch;
			S31_angle = S31_pitch;
			S32_angle = S32_pitch;
			S33_angle = S33_pitch;
			break;

		case YAW:
			angle_ekf = yaw_ekf;
			angle_rate = yaw_rate;
			angle_bias = yaw_bias;
			angle_acc = yaw_acc;
			gyro = gyroZ;
			sa = 5e2;
			Qa = 5e7;
			Qg = 1e1;

			S11_angle = S11_yaw;
			S12_angle = S12_yaw;
			S13_angle = S13_yaw;
			S21_angle = S21_yaw;
			S22_angle = S22_yaw;
			S23_angle = S23_yaw;
			S31_angle = S31_yaw;
			S32_angle = S32_yaw;
			S33_angle = S33_yaw;
			break;

	}


    angle_ekf = (angle_ekf) + st*(angle_rate);

    S11_angle = S11_angle + sa + S31_angle*st + (st*st*(S13_angle + S33_angle*st))/st;
    S12_angle = S12_angle + S32_angle*st;
    S13_angle = S13_angle + S33_angle*st;

    S21_angle = S21_angle + S23_angle*(st);
    S22_angle = S22_angle + sb;
    //S23_angle = S23_angle;

    S31_angle = S31_angle + S33_angle*(st);
    //S32_angle = S32_angle;
    S33_angle = S33_angle + sr;

    //ANGLE CORRECTION
    float A = (Qa*Qg + Qa*S22_angle + Qa*S23_angle + Qa*S32_angle + Qa*S33_angle + Qg*S11_angle + S11_angle*S22_angle - S12_angle*S21_angle + S11_angle*S23_angle - S13_angle*S21_angle + S11_angle*S32_angle - S12_angle*S31_angle + S11_angle*S33_angle - S13_angle*S31_angle);
    float Kt11_att = 1 - (Qa*(Qg + S22_angle + S23_angle + S32_angle + S33_angle))/A;
    float Kt12_att = (Qa*(S12_angle + S13_angle))/A;
    float Kt21_att = (Qg*S21_angle + S21_angle*S32_angle - S22_angle*S31_angle + S21_angle*S33_angle - S23_angle*S31_angle)/A;
    float Kt22_att = (Qa*S22_angle + Qa*S23_angle + S11_angle*S22_angle - S12_angle*S21_angle + S11_angle*S23_angle - S13_angle*S21_angle)/A;
    float Kt31_att = (Qg*S31_angle - S21_angle*S32_angle + S22_angle*S31_angle - S21_angle*S33_angle + S23_angle*S31_angle)/A;
    float Kt32_att = (Qa*S32_angle + Qa*S33_angle + S11_angle*S32_angle - S12_angle*S31_angle + S11_angle*S33_angle - S13_angle*S31_angle)/A;

    angle_ekf = (angle_ekf) + Kt11_att*((angle_acc) - (angle_ekf)) - Kt12_att*((angle_bias) - (gyro) + (angle_rate));


    angle_bias = (angle_bias) + Kt21_att*((angle_acc) - (angle_ekf)) - Kt22_att*((angle_bias) - (gyro) + (angle_rate));


    angle_rate = (angle_rate) + Kt31_att*((angle_acc) - (angle_ekf)) - Kt32_att*((angle_bias) - (gyro) + (angle_rate));


    S11_angle = - S11_angle*(Kt11_att - 1) - Kt12_att*S21_angle - Kt12_att*S31_angle;


    S12_angle = - S12_angle*(Kt11_att - 1) - Kt12_att*S22_angle - Kt12_att*S32_angle;


    S13_angle = - S13_angle*(Kt11_att - 1) - Kt12_att*S23_angle - Kt12_att*S33_angle;


    S21_angle = - S21_angle*(Kt22_att - 1) - Kt21_att*S11_angle - Kt22_att*S31_angle;


    S22_angle = - S22_angle*(Kt22_att - 1) - Kt21_att*S12_angle - Kt22_att*S32_angle;


    S23_angle = - S23_angle*(Kt22_att - 1) - Kt21_att*S13_angle - Kt22_att*S33_angle;


    S31_angle = - S31_angle*(Kt32_att - 1) - Kt31_att*S11_angle - Kt32_att*S21_angle;


    S32_angle = - S32_angle*(Kt32_att - 1) - Kt31_att*S12_angle - Kt32_att*S22_angle;


    S33_angle = - S33_angle*(Kt32_att - 1) - Kt31_att*S13_angle - Kt32_att*S23_angle;

    switch(euler_angle) {
    		case ROLL:
    			 roll_ekf = angle_ekf ;
    			 roll_rate = angle_rate;
    			 roll_bias = angle_bias;
    			 roll_acc = angle_acc ;

    			 S11_roll = S11_angle;
    			 S12_roll = S12_angle;
    			 S13_roll = S13_angle;
    			 S21_roll = S21_angle;
    			 S22_roll = S22_angle;
    			 S23_roll = S23_angle;
    			 S31_roll = S31_angle;
    			 S32_roll = S32_angle;
    			 S33_roll = S33_angle;
    			break;

    		case PITCH:
    			 pitch_ekf = angle_ekf ;
    			 pitch_rate = angle_rate;
    			 pitch_bias = angle_bias;
    			 pitch_acc = angle_acc ;

    			 S11_pitch = S11_angle;
    			 S12_pitch = S12_angle;
    			 S13_pitch = S13_angle;
    			 S21_pitch = S21_angle;
    			 S22_pitch = S22_angle;
    			 S23_pitch = S23_angle;
    			 S31_pitch = S31_angle;
    			 S32_pitch = S32_angle;
    			 S33_pitch = S33_angle;
    			break;

    		case YAW:
    			 yaw_ekf = angle_ekf ;
    			 yaw_rate = angle_rate;
    			 yaw_bias = angle_bias;
    			 yaw_acc = angle_acc ;

    			 S11_yaw = S11_angle;
    			 S12_yaw = S12_angle;
    			 S13_yaw = S13_angle;
    			 S21_yaw = S21_angle;
    			 S22_yaw = S22_angle;
    			 S23_yaw = S23_angle;
    			 S31_yaw = S31_angle;
    			 S32_yaw = S32_angle;
    			 S33_yaw = S33_angle;
    			break;

    }
}

void Kalman_Filtresi::EKF_Alt() {
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

	  float A = (Qb*Qs + Qb*S11_alt + Qs*S11_alt + Qs*S13_alt + Qs*S31_alt + Qs*S33_alt + S11_alt*S33_alt - S13_alt*S31_alt);

	  //ALT CORRECTION
	  float Kt11 = (Qs*(S11_alt + S13_alt));
	  Kt11 = Kt11/A;
	  float Kt12 = (Qb*S11_alt + S11_alt*S33_alt - S13_alt*S31_alt);
	  Kt12 = Kt12/A;

	  float Kt21 = (Qs*S21_alt + Qs*S23_alt + S11_alt*S23_alt - S13_alt*S21_alt);
	  Kt21 = Kt21/A;
	  float Kt22 = (Qb*S21_alt - S11_alt*S23_alt + S13_alt*S21_alt + S21_alt*S33_alt - S23_alt*S31_alt);
	  Kt22 = Kt22/A;

	  float Kt31 = (Qs*S31_alt + Qs*S33_alt + S11_alt*S33_alt - S13_alt*S31_alt);
	  Kt31 = Kt31/A;
	  float Kt32 = (Qb*S31_alt - S11_alt*S33_alt + S13_alt*S31_alt);
	  Kt32 = Kt32/A;

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
}

void Kalman_Filtresi::EKF_Cam() {
	  //X Position Estimation
	  //camx = cam_filt.Run(camx);
	  xpos = (xpos) + st*(vx) + (accXm*st*st)/2;
	  vx = (vx) + accXm*(st);

	  S11_x = S11_x + spx + S21_x*st + (st*st*(S12_x + S22_x*st))/st;
	  S12_x = S12_x + S22_x*st;
	  S21_x = S21_x + svx + S22_x*(st);
	  //S22_x = S22_x;


	  //X Position Correction
	  float Kt11 = S11_x/(Qc + S11_x);
	  float Kt21 = S21_x/(Qc + S11_x);



	  xpos = (xpos) + (Kt11)*(camx - (xpos));

	  vx = (vx) + (Kt21)*(camx - (xpos));


	  S11_x = -S11_x*((Kt11) - 1);

	  S12_x = -S12_x*((Kt11) - 1);

	  S21_x = S21_x - S11_x*(Kt21);

	  S22_x = S22_x - S12_x*(Kt21);


}
void Kalman_Filtresi::Run() {



  float A;
  float accX = acc[0];
  float accY = acc[1];
  float accZ = acc[2];

  float gyroX = gyro[0];
  float gyroY = gyro[1];
  float gyroZ = gyro[2];

    //---IMU KİSMİ----
    //=================================


  //roll_acc-=-0.67;	pitch_acc-=-1.15;	//İvmeölçer ile okunan hata değerleri offsetlemesi


  //pitch_acc = lpf(pitch_acc);
  //roll_acc = lpf(roll_acc);


  //pitch_gyro = gyroY * st;
  //roll_gyro =  gyroX * st;


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
    
    if(gyro_ready) {

  //float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);

  //pitch_acc =  asin(accX/acctop)*rad2deg + PITCH_OFFSET;
  //roll_acc  =  asin(accY/acctop)*rad2deg + ROLL_OFFSET;

  	//float alpha = 0.998;
  	//float alpha = 0.998;
    //pitch_comp=(pitch_gyro+pitch_eski)*alpha+pitch_acc*(1-alpha);	//Tümleyen filtre
    //roll_comp =(roll_gyro+roll_eski)*alpha+roll_acc*(1-alpha);		//Tümleyen filtre

    EKF_Attitude(ROLL);
    EKF_Attitude(PITCH);
    EKF_Attitude(YAW);

    EKF_Alt();

    EKF_Cam();

    }


    else {

    	for(int i=0; i<2000; i++) {
    		  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);

    		  float pitch_acc =  asin(accX/acctop)*rad2deg;
    		  float roll_acc  =  asin(accY/acctop)*rad2deg ;

        	ROLL_OFFSET += roll_acc;
        	PITCH_OFFSET += pitch_acc;
    	}

    	ROLL_OFFSET  = -1*  ROLL_OFFSET  / 2000;
    	PITCH_OFFSET = -1 * PITCH_OFFSET / 2000;
    	gyro_ready = true;
    }


	//pitch_eski=pitch_comp;
	//roll_eski=roll_comp;

    state.angles[0] = roll_ekf;
    state.angles[1] = pitch_ekf;
    state.angles[2] = -1*yaw_ekf;

    state.rates[0] = roll_rate;
    state.rates[1] = pitch_rate;
    state.rates[2] = yaw_rate;

    state.bias[0] = roll_bias;
    state.bias[1] = pitch_bias;
    state.bias[2] = yaw_bias;

}

Kalman_Filtresi::~Kalman_Filtresi() {}
