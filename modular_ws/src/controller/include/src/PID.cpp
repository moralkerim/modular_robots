#include "PID.hpp"

PID::PID() {};

double PID::P_Angle(double alpha_des, double alpha, double Kp_angle) {
	double P;
	double e = alpha_des - alpha;
	P = Kp_angle*e;
    return P;

}


double PID::PD_Rate(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd) {

	e_roll = alpha_dot_des - alpha_dot;
  double e_roll_int = e_roll;

  if((int)pd_roll_buf != (int)pd_roll_sat_buf) {
    if(sgn(e_roll) == sgn(pd_roll_sat_buf)) {
      e_roll_int = 0;
    }
  }

  	de_filt = N * (Kd * e_roll - de_int);
  	de_int += de_filt*st;

	de = e_roll - e_eski_roll;
	e_eski_roll = e_roll;

  ie_roll += e_roll_int*st;

  ie_roll_sat = ie_roll;
	

	P = Kp*e_roll; D = de_filt; I = Ki * ie_roll_sat;
	//D = lpf.update(D);
	pd = P + I + D;
  	pd_roll_buf = pd;
	pd  = Sat(pd,  300, -300);
	pd_roll_sat_buf = pd;
    return pd;

}

void PID::reset() {
	ie_roll_sat = 0;
	de_filt = 0;
	de_int = 0;
}


double PID::P_Rate_Yaw(double alpha_dot_des, double alpha_dot, double Kp) {
	double P;
	double e_yaw = alpha_dot_des - alpha_dot;	
	P = Kp*e_yaw;
	P    = Sat(P,    150, -150);

    return P;

}

double PID::sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

 double PID::Sat(double pwm, int max, int min, int thr) {
	double pwm_out;

	if(thr > 1020) {
		if(pwm > max) {
			pwm_out = max;
		}

		else if (pwm < min) {
			pwm_out = min;
		}

		else {
			pwm_out = pwm;
		}


	}

	else {
		pwm_out = 1000;
	}
	return pwm_out;
}

 double PID::Sat(double pwm, int max, int min) {
	double pwm_out;

		if(pwm > max) {
			pwm_out = max;
		}

		else if (pwm < min) {
			pwm_out = min;
		}

		else {
			pwm_out = pwm;
		}




	return pwm_out;
}

float PID::pwm2ang(unsigned short int pwm) {
	int dead_zone = 5;
	int in_min  = 1000;
	int in_max  = 2000;
	int out_min = -30;
	int out_max  = 30;
	unsigned short int pwm_out;

	if(pwm > 1500 - dead_zone && pwm < 1500 + dead_zone) {
		pwm_out = 1500;
	}

	else {
		pwm_out = pwm;
	}

	return (pwm_out - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float PID::pwm2rate(unsigned short int pwm) {
	int in_min  = 1000;
	int in_max  = 2000;
	int out_min = -100;
	int out_max  = 100;

	return -1 * ((pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

//Convert pwm to motor speed for simulation
float PID::pwm2mot(unsigned short int pwm, int dir) {
	float in_min  = 1000;
	float in_max  = 2000;
	float out_min = 0;
	float out_max  = 1326;

	return (float)(dir) * ((float)pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned int PID::F2thr(float F) {
	float kf = 5.074714371861032e-08;
	float max_rpm = 17591;
	float Fm = F/4;
	float wh = sqrt(Fm/kf);

	unsigned int thr = (wh - 0) * (2000 - 1000) / (max_rpm - 0) + 1000;
	return thr;
}

PID::~PID() {};
