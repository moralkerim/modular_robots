#include <Kalman.hpp>
#include <math.h>


Kalman_Filtresi::Kalman_Filtresi()  {

}
void Kalman_Filtresi::PredictUpdatePos(pos_axis axis) {
	float pos,vel,a,b,accm,pos_gps,vgps,S1_1,S1_2,S1_3,S1_4,S2_1,S2_2,S2_3,S2_4,S3_1,S3_2,S3_3,S3_4,S4_1, S4_2, S4_3, S4_4;
	float angle;
	float deg2rad = M_PI/180.0;

	switch (axis) {
		case x_axis:
			pos = x;
			a = apx;
			vel = vx;
			b = bax;

			angle = -pitch_ekf*deg2rad;

			pos_gps = xgps;
			vgps   = vgpsx;
			accm		= accx;

			S1_1 = Sp1_1x;
			S1_2 = Sp1_2x;
			S1_3 = Sp1_3x;
			S1_4 = Sp1_4x;
			S2_1 = Sp2_1x;
			S2_2 = Sp2_2x;
			S2_3 = Sp2_3x;
			S2_4 = Sp2_4x;
			S3_1 = Sp3_1x;
			S3_2 = Sp3_2x;
			S3_3 = Sp3_3x;
			S3_4 = Sp3_4x;
			S4_1 = Sp4_1x;
			S4_2 = Sp4_2x;
			S4_3 = Sp4_3x;
			S4_4 = Sp4_4x;
			break;

		case y_axis:
			pos = y;
			a = apy;
			vel = vy;
			b = bay;

			angle = roll_ekf*deg2rad;

			pos_gps = ygps;
			vgps   = vgpsy;
			accm		= accy;

			S1_1 = Sp1_1y;
			S1_2 = Sp1_2y;
			S1_3 = Sp1_3y;
			S1_4 = Sp1_4y;
			S2_1 = Sp2_1y;
			S2_2 = Sp2_2y;
			S2_3 = Sp2_3y;
			S2_4 = Sp2_4y;
			S3_1 = Sp3_1y;
			S3_2 = Sp3_2y;
			S3_3 = Sp3_3y;
			S3_4 = Sp3_4y;
			S4_1 = Sp4_1y;
			S4_2 = Sp4_2y;
			S4_3 = Sp4_3y;
			S4_4 = Sp4_4y;
			break;

	}
	pos =(a*pos_st*pos_st)/2 + (vel)*pos_st + (pos);
	vel =                   (vel) + pos_st*a;

//	if(armed) {
		//float g = 9.81;
		//a = g*angle;
//	}

	//b =                                (b);

	S1_1=S1_1 + sx + S1_2*pos_st + S2_1*pos_st + (S1_3*pos_st*pos_st)/2 + S2_2*pos_st*pos_st + (S2_3*pos_st*pos_st*pos_st)/2 + (S3_1*pos_st*pos_st)/2 + (S3_2*pos_st*pos_st*pos_st)/2 + (S3_3*pos_st*pos_st*pos_st*pos_st)/4;
	S1_2=S1_2 + S1_3*pos_st + S2_2*pos_st + S2_3*pos_st*pos_st + (S3_2*pos_st*pos_st)/2 + (S3_3*pos_st*pos_st*pos_st)/2;
	S1_3=0;
	S1_4=S1_4 + S2_4*pos_st + (S3_4*pos_st*pos_st)/2;
	S2_1=S2_1 + (pos_st*pos_st*(S2_3 + S3_3*pos_st))/2 + S3_1*pos_st + pos_st*(S2_2 + S3_2*pos_st);
	S2_2=S2_2 + sv + S3_2*pos_st + pos_st*(S2_3 + S3_3*pos_st);
	S2_3=0;
	S2_4=S2_4 + S3_4*pos_st;
	S3_1=0;
	S3_2=0;
	S3_3=sa;
	S3_4=0;
	S4_1=S4_1 + S4_2*pos_st + (S4_3*pos_st*pos_st)/2;
	S4_2=S4_2 + S4_3*pos_st;
	S4_3=0;
	S4_4=S4_4 + sb;

	float A = (Qap*Qgps*Qgps_v + Qap*Qgps*S2_2 + Qap*Qgps_v*S1_1 + Qgps*Qgps_v*S3_3 + Qgps*Qgps_v*S3_4 + Qgps*Qgps_v*S4_3 + Qgps*Qgps_v*S4_4 + Qap*S1_1*S2_2 - Qap*S1_2*S2_1 + Qgps*S2_2*S3_3 - Qgps*S2_3*S3_2 + Qgps*S2_2*S3_4 - Qgps*S2_4*S3_2 + Qgps*S2_2*S4_3 - Qgps*S2_3*S4_2 + Qgps*S2_2*S4_4 - Qgps*S2_4*S4_2 + Qgps_v*S1_1*S3_3 - Qgps_v*S1_3*S3_1 + Qgps_v*S1_1*S3_4 - Qgps_v*S1_4*S3_1 + Qgps_v*S1_1*S4_3 - Qgps_v*S1_3*S4_1 + Qgps_v*S1_1*S4_4 - Qgps_v*S1_4*S4_1 + S1_1*S2_2*S3_3 - S1_1*S2_3*S3_2 - S1_2*S2_1*S3_3 + S1_2*S2_3*S3_1 + S1_3*S2_1*S3_2 - S1_3*S2_2*S3_1 + S1_1*S2_2*S3_4 - S1_1*S2_4*S3_2 - S1_2*S2_1*S3_4 + S1_2*S2_4*S3_1 + S1_4*S2_1*S3_2 - S1_4*S2_2*S3_1 + S1_1*S2_2*S4_3 - S1_1*S2_3*S4_2 - S1_2*S2_1*S4_3 + S1_2*S2_3*S4_1 + S1_3*S2_1*S4_2 - S1_3*S2_2*S4_1 + S1_1*S2_2*S4_4 - S1_1*S2_4*S4_2 - S1_2*S2_1*S4_4 + S1_2*S2_4*S4_1 + S1_4*S2_1*S4_2 - S1_4*S2_2*S4_1);
	float Kt11=1 - (Qap*Qgps*Qgps_v + Qap*Qgps*S2_2 + Qgps*Qgps_v*S3_3 + Qgps*Qgps_v*S3_4 + Qgps*Qgps_v*S4_3 + Qgps*Qgps_v*S4_4 + Qgps*S2_2*S3_3 - Qgps*S2_3*S3_2 + Qgps*S2_2*S3_4 - Qgps*S2_4*S3_2 + Qgps*S2_2*S4_3 - Qgps*S2_3*S4_2 + Qgps*S2_2*S4_4 - Qgps*S2_4*S4_2)/A;
	float Kt12=(Qgps*(Qgps_v*S1_3 + Qgps_v*S1_4 - S1_2*S2_3 + S1_3*S2_2 - S1_2*S2_4 + S1_4*S2_2))/A;
	float Kt13=(Qgps*(Qap*S1_2 + S1_2*S3_3 - S1_3*S3_2 + S1_2*S3_4 - S1_4*S3_2 + S1_2*S4_3 - S1_3*S4_2 + S1_2*S4_4 - S1_4*S4_2))/A;
	float Kt21=(Qgps_v*(Qap*S2_1 + S2_1*S3_3 - S2_3*S3_1 + S2_1*S3_4 - S2_4*S3_1 + S2_1*S4_3 - S2_3*S4_1 + S2_1*S4_4 - S2_4*S4_1))/A;
	float Kt22=(Qgps_v*(Qgps*S2_3 + Qgps*S2_4 + S1_1*S2_3 - S1_3*S2_1 + S1_1*S2_4 - S1_4*S2_1))/A;
	float Kt23=1 - (Qap*Qgps*Qgps_v + Qap*Qgps_v*S1_1 + Qgps*Qgps_v*S3_3 + Qgps*Qgps_v*S3_4 + Qgps*Qgps_v*S4_3 + Qgps*Qgps_v*S4_4 + Qgps_v*S1_1*S3_3 - Qgps_v*S1_3*S3_1 + Qgps_v*S1_1*S3_4 - Qgps_v*S1_4*S3_1 + Qgps_v*S1_1*S4_3 - Qgps_v*S1_3*S4_1 + Qgps_v*S1_1*S4_4 - Qgps_v*S1_4*S4_1)/A;
	float Kt31=(Qap*Qgps_v*S3_1 - Qap*S2_1*S3_2 + Qap*S2_2*S3_1 + Qgps_v*S3_1*S4_3 - Qgps_v*S3_3*S4_1 + Qgps_v*S3_1*S4_4 - Qgps_v*S3_4*S4_1 - S2_1*S3_2*S4_3 + S2_1*S3_3*S4_2 + S2_2*S3_1*S4_3 - S2_2*S3_3*S4_1 - S2_3*S3_1*S4_2 + S2_3*S3_2*S4_1 - S2_1*S3_2*S4_4 + S2_1*S3_4*S4_2 + S2_2*S3_1*S4_4 - S2_2*S3_4*S4_1 - S2_4*S3_1*S4_2 + S2_4*S3_2*S4_1)/A;
	float Kt32=(Qgps*Qgps_v*S3_3 + Qgps*Qgps_v*S3_4 + Qgps*S2_2*S3_3 - Qgps*S2_3*S3_2 + Qgps*S2_2*S3_4 - Qgps*S2_4*S3_2 + Qgps_v*S1_1*S3_3 - Qgps_v*S1_3*S3_1 + Qgps_v*S1_1*S3_4 - Qgps_v*S1_4*S3_1 + S1_1*S2_2*S3_3 - S1_1*S2_3*S3_2 - S1_2*S2_1*S3_3 + S1_2*S2_3*S3_1 + S1_3*S2_1*S3_2 - S1_3*S2_2*S3_1 + S1_1*S2_2*S3_4 - S1_1*S2_4*S3_2 - S1_2*S2_1*S3_4 + S1_2*S2_4*S3_1 + S1_4*S2_1*S3_2 - S1_4*S2_2*S3_1)/A;
	float Kt33=(Qap*Qgps*S3_2 + Qap*S1_1*S3_2 - Qap*S1_2*S3_1 + Qgps*S3_2*S4_3 - Qgps*S3_3*S4_2 + Qgps*S3_2*S4_4 - Qgps*S3_4*S4_2 + S1_1*S3_2*S4_3 - S1_1*S3_3*S4_2 - S1_2*S3_1*S4_3 + S1_2*S3_3*S4_1 + S1_3*S3_1*S4_2 - S1_3*S3_2*S4_1 + S1_1*S3_2*S4_4 - S1_1*S3_4*S4_2 - S1_2*S3_1*S4_4 + S1_2*S3_4*S4_1 + S1_4*S3_1*S4_2 - S1_4*S3_2*S4_1)/A;
	float Kt41=(Qap*Qgps_v*S4_1 - Qap*S2_1*S4_2 + Qap*S2_2*S4_1 - Qgps_v*S3_1*S4_3 + Qgps_v*S3_3*S4_1 - Qgps_v*S3_1*S4_4 + Qgps_v*S3_4*S4_1 + S2_1*S3_2*S4_3 - S2_1*S3_3*S4_2 - S2_2*S3_1*S4_3 + S2_2*S3_3*S4_1 + S2_3*S3_1*S4_2 - S2_3*S3_2*S4_1 + S2_1*S3_2*S4_4 - S2_1*S3_4*S4_2 - S2_2*S3_1*S4_4 + S2_2*S3_4*S4_1 + S2_4*S3_1*S4_2 - S2_4*S3_2*S4_1)/A;
	float Kt42=(Qgps*Qgps_v*S4_3 + Qgps*Qgps_v*S4_4 + Qgps*S2_2*S4_3 - Qgps*S2_3*S4_2 + Qgps*S2_2*S4_4 - Qgps*S2_4*S4_2 + Qgps_v*S1_1*S4_3 - Qgps_v*S1_3*S4_1 + Qgps_v*S1_1*S4_4 - Qgps_v*S1_4*S4_1 + S1_1*S2_2*S4_3 - S1_1*S2_3*S4_2 - S1_2*S2_1*S4_3 + S1_2*S2_3*S4_1 + S1_3*S2_1*S4_2 - S1_3*S2_2*S4_1 + S1_1*S2_2*S4_4 - S1_1*S2_4*S4_2 - S1_2*S2_1*S4_4 + S1_2*S2_4*S4_1 + S1_4*S2_1*S4_2 - S1_4*S2_2*S4_1)/A;
	float Kt43=(Qap*Qgps*S4_2 + Qap*S1_1*S4_2 - Qap*S1_2*S4_1 - Qgps*S3_2*S4_3 + Qgps*S3_3*S4_2 - Qgps*S3_2*S4_4 + Qgps*S3_4*S4_2 - S1_1*S3_2*S4_3 + S1_1*S3_3*S4_2 + S1_2*S3_1*S4_3 - S1_2*S3_3*S4_1 - S1_3*S3_1*S4_2 + S1_3*S3_2*S4_1 - S1_1*S3_2*S4_4 + S1_1*S3_4*S4_2 + S1_2*S3_1*S4_4 - S1_2*S3_4*S4_1 - S1_4*S3_1*S4_2 + S1_4*S3_2*S4_1)/A;

	pos = pos - Kt13*(vel - vgps) - Kt11*(pos - pos_gps) - Kt12*(a - accm + b);

	vel = vel - Kt23*(vel - vgps) - Kt21*(pos - pos_gps) - Kt22*(a - accm + b);

	a = a - Kt33*(vel - vgps) - Kt31*(pos - pos_gps) - Kt32*(a - accm + b);

	b = b - Kt43*(vel - vgps) - Kt41*(pos - pos_gps) - Kt42*(a - accm + b);

	S1_1=- S1_1*(Kt11 - 1) - Kt13*S2_1 - Kt12*S3_1 - Kt12*S4_1;
	S1_2=- S1_2*(Kt11 - 1) - Kt13*S2_2 - Kt12*S3_2 - Kt12*S4_2;
	S1_3=- S1_3*(Kt11 - 1) - Kt13*S2_3 - Kt12*S3_3 - Kt12*S4_3;
	S1_4=- S1_4*(Kt11 - 1) - Kt13*S2_4 - Kt12*S3_4 - Kt12*S4_4;
	S2_1=- S2_1*(Kt23 - 1) - Kt21*S1_1 - Kt22*S3_1 - Kt22*S4_1;
	S2_2=- S2_2*(Kt23 - 1) - Kt21*S1_2 - Kt22*S3_2 - Kt22*S4_2;
	S2_3=- S2_3*(Kt23 - 1) - Kt21*S1_3 - Kt22*S3_3 - Kt22*S4_3;
	S2_4=- S2_4*(Kt23 - 1) - Kt21*S1_4 - Kt22*S3_4 - Kt22*S4_4;
	S3_1=- S3_1*(Kt32 - 1) - Kt31*S1_1 - Kt33*S2_1 - Kt32*S4_1;
	S3_2=- S3_2*(Kt32 - 1) - Kt31*S1_2 - Kt33*S2_2 - Kt32*S4_2;
	S3_3=- S3_3*(Kt32 - 1) - Kt31*S1_3 - Kt33*S2_3 - Kt32*S4_3;
	S3_4=- S3_4*(Kt32 - 1) - Kt31*S1_4 - Kt33*S2_4 - Kt32*S4_4;
	S4_1=- S4_1*(Kt42 - 1) - Kt41*S1_1 - Kt43*S2_1 - Kt42*S3_1;
	S4_2=- S4_2*(Kt42 - 1) - Kt41*S1_2 - Kt43*S2_2 - Kt42*S3_2;
	S4_3=- S4_3*(Kt42 - 1) - Kt41*S1_3 - Kt43*S2_3 - Kt42*S3_3;
	S4_4=- S4_4*(Kt42 - 1) - Kt41*S1_4 - Kt43*S2_4 - Kt42*S3_4;

	switch (axis) {
		case x_axis:
			x = pos;
			vx = vel;
			apx = a;
			bax = b;

			 Sp1_1x = S1_1;
			 Sp1_2x = S1_2;
			 Sp1_3x = S1_3 ;
			 Sp1_4x = S1_4 ;
			 Sp2_1x = S2_1 ;
			 Sp2_2x = S2_2;
			 Sp2_3x = S2_3 ;
			 Sp2_4x = S2_4 ;
			 Sp3_1x = S3_1 ;
			 Sp3_2x = S3_2 ;
			 Sp3_3x = S3_3 ;
			 Sp3_4x = S3_4 ;
			 Sp4_1x = S4_1;
			 Sp4_2x = S4_2;
			 Sp4_3x = S4_3 ;
			 Sp4_4x = S4_4 ;
			break;

		case y_axis:
			y = pos;
			vy = vel;
			apy = a;
			bay = b;

			 Sp1_1y = S1_1;
			 Sp1_2y = S1_2;
			 Sp1_3y = S1_3 ;
			 Sp1_4y = S1_4 ;
			 Sp2_1y = S2_1 ;
			 Sp2_2y = S2_2;
			 Sp2_3y = S2_3 ;
			 Sp2_4y = S2_4 ;
			 Sp3_1y = S3_1 ;
			 Sp3_2y = S3_2 ;
			 Sp3_3y = S3_3 ;
			 Sp3_4y = S3_4 ;
			 Sp4_1y = S4_1;
			 Sp4_2y = S4_2;
			 Sp4_3y = S4_3 ;
			 Sp4_4y = S4_4 ;
			break;

	}

}

void Kalman_Filtresi::EKF_Pos() {

	if(pos_ekf_counter == POS_EKF_RATE) { //50 Hz
		accx = acc_pos_x;
		accy = acc_pos_y;
		acc_pos_x_med = 0;
		pos_ekf_counter = 0;
		gps_ekf_counter++;

		if(gps_fixed) {

			if(	gps_ekf_counter >= 10) {	//5 Hz
				NED2Body();

				gps_ekf_counter = 0;
				Qgps = 4.0;
				Qgps_v = 80;
			}

			else {
				Qgps = 1.0e9;
				Qgps_v = 1.0e9;
			}

		}

		else {
				Qgps = 1.0e9;
				Qgps_v = 1.0e9;
					}

		PredictUpdatePos(x_axis);
		PredictUpdatePos(y_axis);


	}


}


void Kalman_Filtresi::EKF_Attitude(euler_angle euler_angle, bool update_enable) {
	  float accX = acc[0];
	  float accY = acc[1];
	  float accZ = acc[2];

	  float gyroX = gyro[0]*deg2rad;
	  float gyroY = gyro[1]*deg2rad;
	  float gyroZ = gyro[2]*deg2rad;

	  //gyroZ = lpf_yaw.Run(gyroZ);

	  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);



	  pitch_acc =  asin(accX/g) + PITCH_OFFSET*deg2rad;
	  roll_acc  =  atan(accY/accZ) + ROLL_OFFSET*deg2rad;

	 // roll_gyro_comp  = gyro[0] * st;
	 // pitch_gyro_comp = gyro[1] * st;

	float angle_ekf, angle_rate, angle_bias, angle_acc, gyro;
	float S11_angle, S12_angle, S13_angle, S21_angle, S22_angle, S23_angle, S31_angle, S32_angle, S33_angle ;


	switch(euler_angle) {
	default:
		Qa = 5e4;
		break;

	case YAW:
		Qa = 5e8;
		break;

	}


	//ANGLE PREDICTION
	switch(euler_angle) {
		case ROLL:
			angle_ekf = roll_ekf;
			angle_rate = roll_rate;
			angle_bias = roll_bias;
			angle_acc = roll_acc;
			gyro = gyroX;

			S11_angle = S11_roll;
			S12_angle = S12_roll;
			S21_angle = S21_roll;
			S22_angle = S22_roll;
			break;

		case PITCH:
			angle_ekf = pitch_ekf;
			angle_rate = pitch_rate;
			angle_bias = pitch_bias;
			angle_acc = pitch_acc;
			gyro = gyroY;

			S11_angle = S11_pitch;
			S12_angle = S12_pitch;
			S21_angle = S21_pitch;
			S22_angle = S22_pitch;
			break;

		case YAW:
			angle_ekf = yaw_ekf;
			angle_rate = yaw_rate;
			angle_bias = yaw_bias;
			angle_acc = yaw_acc*deg2rad;
			gyro = gyroZ;


			S11_angle = S11_yaw;
			S12_angle = S12_yaw;
			S21_angle = S21_yaw;
			S22_angle = S22_yaw;
			break;

	}

    angle_ekf = angle_ekf - angle_bias*st + gyro*st;

    if(update_enable) {

		float CS11 = -S22_angle*st;
		float CS12 = CS11 + S12_angle;

		S11_angle = S11_angle + sa - CS12*st - S21_angle*st;
		S12_angle = CS12;
		S21_angle = CS11 + S21_angle;
		S22_angle = S22_angle + sb;

		//ANGLE CORRECTION
		float CK = 1/(Qa + S11_angle);

		float Kt11 = CK*S11_angle;
		float Kt21 = CK*S21_angle;

		float Cx11 = angle_acc - angle_ekf;

		angle_ekf  = angle_ekf  + Cx11*Kt11;
		angle_bias = angle_bias + Cx11*Kt21;
		angle_rate = gyro - angle_bias;

		CS11 = Kt11 - 1;

		S11_angle = -CS11*S11_angle;
		S12_angle = -CS11*S12_angle;
		S21_angle = S21_angle - Kt21*S11_angle;
		S22_angle = S22_angle - Kt21*S12_angle;
    }
    switch(euler_angle) {
    		case ROLL:
    			 roll_ekf = angle_ekf ;
    			 roll_rate = angle_rate;
    			 roll_bias = angle_bias;
    			 roll_acc = angle_acc ;

    			 S11_roll = S11_angle;
    			 S12_roll = S12_angle;
    			 S21_roll = S21_angle;
    			 S22_roll = S22_angle;
    			break;

    		case PITCH:
    			 pitch_ekf = angle_ekf ;
    			 pitch_rate = angle_rate;
    			 pitch_bias = angle_bias;
    			 pitch_acc = angle_acc ;

    			 S11_pitch = S11_angle;
    			 S12_pitch = S12_angle;
    			 S21_pitch = S21_angle;
    			 S22_pitch = S22_angle;
    			break;

    		case YAW:
    			 yaw_ekf = angle_ekf ;
    			 //yaw_rate = angle_rate;
    			 yaw_rate = gyroZ;
    			 yaw_bias = angle_bias;
    			 yaw_acc = angle_acc ;

    			 S11_yaw = S11_angle;
    			 S12_yaw = S12_angle;
    			 S21_yaw = S21_angle;
    			 S22_yaw = S22_angle;
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

/*
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
*/
void Kalman_Filtresi::NED2Body() {
	float deg2rad = M_PI/180.0;

	float yaw   = -yaw_ekf*deg2rad;
	//float roll  = roll_ekf*deg2rad;
	//float pitch = pitch_ekf*deg2rad;

	float DCM11 = cos(yaw);
	float DCM22 = DCM11;

	float DCM12 = sin(yaw);
	float DCM21 = -DCM12;

	_xbody = xbody;
	_ybody = ybody;

	xbody = DCM11*xned + DCM21*yned;
	ybody = DCM12*xned + DCM22*yned;


	//xbody = vel_gps_filt.Run(xbody);
	//ybody = vel_gps_filt.Run(ybody);


	/*
	vgpsx = DCM11*vgpsxned + DCM21*vgpsyned;
	vgpsy = DCM12*vgpsxned + DCM22*vgpsyned;
	*/

	vgpsx = (xbody - _xbody) / 0.2;
	vgpsy = (ybody - _ybody) / 0.2;

	//vgpsx = vel_gps_filt.Run(vgpsx);
	//vgpsy = vel_gps_filt.Run(vgpsy);

	xgps = xbody;
	ygps = ybody;

}
void Kalman_Filtresi::Run() {

  acc_pos_x_med += acc_pos_x;
  float accX = acc[0];
  float accY = acc[1];
  float accZ = acc[2];




//    if(gyro_ready) {

    pos_ekf_counter++;
    EKF_Attitude(ROLL, ekf_update);
    EKF_Attitude(PITCH, ekf_update);
    EKF_Attitude(YAW, ekf_update);



    //EKF_Alt();
    //EKF_Pos();

    //EKF_Cam();

//    }


//    else {
//    	uint16_t SAMPLE_NUMBER = 5000;
//    	for(int i=0; i<SAMPLE_NUMBER; i++) {
//    		  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);
//
//    		  float pitch_acc =  asin(accX/acctop)*rad2deg;
//    		  float roll_acc  =  asin(accY/acctop)*rad2deg ;
//
//        	ROLL_OFFSET += roll_acc;
//        	PITCH_OFFSET += pitch_acc;
//    	}
//
//    	ROLL_OFFSET  = -1*  ROLL_OFFSET  / SAMPLE_NUMBER;
//    	PITCH_OFFSET = -1 * PITCH_OFFSET / SAMPLE_NUMBER;
//
//    	gyro_ready = true;
//    }


	pitch_eski=pitch_comp;
	roll_eski=roll_comp;

    state.angles[0] = roll_ekf*rad2deg;
    state.angles[1] = pitch_ekf*rad2deg;
    state.angles[2] = -1*yaw_ekf*rad2deg;

    state.rates[0] = roll_rate*rad2deg;
    state.rates[1] = pitch_rate*rad2deg;
    state.rates[2] = yaw_rate*rad2deg;

    state.bias[0] = roll_bias;
    state.bias[1] = pitch_bias;
    state.bias[2] = yaw_bias;

}

Kalman_Filtresi::~Kalman_Filtresi() {}
