#include "PID.hpp"

PID::PID() {};

float PID::P_Angle(float alpha_des, float alpha, float Kp_angle, float Ki_angle) {
	float P,I;
	e_angle = alpha_des - alpha;
	ie_roll += e_angle*st;
	P = Kp_angle*e_angle;
	I = Ki_angle*ie_roll;
    return P+I;

}

float PID::P_Sqrt(float alpha_des, float alpha, float Kp_angle) {
	e_angle = alpha_des - alpha;
	float abs_e = abs(e_angle);
	uint8_t sign_e = sgn(e_angle);
	float sqrt_e = sqrt(abs_e);
	float P = Kp_angle * sqrt_e + alpha_des;
	P = P * sign_e;
	return P;
}

float PID::PI_Vel(float z0, float z, float v, float Kp_alt, float Ki_alt, unsigned int ch) {
	float P;
	float I;
	float v_des;

	if(ch > 1700) {
		zi = z0 + x_inc;
		v_des = vz_def;
	}

	else if (ch < 1300) {
		zi = z0 - x_inc;
		v_des = -1 * vz_def;
	}

	else {
		zi = z0;
		v_des = 0;
	}

	float e = v_des - v;
	P = Kp_alt*e;

	float ei = z0 - z;
	I = Ki_alt * ei;

	float PI = P + I;
    return PI;

}

float PID::RateFF(float rate_des) {
	float ff = 0.9975*ff_ + 0.0904*rate_des - 0.0904*rate_des_;
	return K_ff*ff;
}


float PID::PID_Rate2(float alpha_dot_des, float alpha_dot, float alpha, float Kp, float Ki, float Kd, float Kp_angle) {
	e_roll = alpha_dot_des - alpha_dot;
	ie_roll_rate += e_roll*st;
	P = Kp * e_roll;



	//I = Ki * (ie_roll * Kp_angle - alpha);
	I = Ki * ie_roll_rate;

	float alpha_dot_dot_des = alpha_dot_des - alpha_dot_des_;
	float alpha_dot_dot = (alpha_dot - alpha_dot_) / st;
	alpha_dot_dot_des = alpha_dot_dot_des / st;
	alpha_dot_dot = d_filt.Run(alpha_dot_dot);

	D = Kd * (-alpha_dot * Kp_angle - alpha_dot_dot);

	//D = d_filt.Run(D);

	/*
  	de_filt = N * (Kd * alpha_dot_des - de_int);
  	de_int += de_filt*st;
  	D = de_filt;
*/

	pd = P + I + D;
  	pd_roll_buf = pd;
	pd  = Sat(pd,  300, -300);
	pd_roll_sat_buf = pd;
	alpha_dot_des_ = alpha_dot_des;
	alpha_dot_ = alpha_dot;
	return pd;
}

float PID::PD_Rate(float alpha_dot_des, float alpha_dot, float Kp, float Ki, float Kd) {

	e_roll = alpha_dot_des - alpha_dot;
  float e_roll_der = - alpha_dot;
  float e_roll_int = e_roll;

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
	ie_roll = 0;
	ie_roll_rate = 0;
	de_filt = 0;
	de_int = 0;
}


float PID::P_Rate_Yaw(float alpha_dot_des, float alpha_dot, float Kp) {
	float P;
	float e_yaw = alpha_dot_des - alpha_dot;
	P = Kp*e_yaw;
	P    = Sat(P,    150, -150);

    return P;

}

uint8_t PID::sgn(float v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

 float PID::Sat(float pwm, int max, int min, int thr) {
	float pwm_out;

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

 float PID::Sat(float pwm, int max, int min) {
	float pwm_out;

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

	/*
	int in_min  = 1160;
	int in_max  = 1850;
	*/
	int out_min = -15;
	int out_max  = 15;
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

	/*
	int in_min  = 1160;
	int in_max  = 1850;
	 */
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
