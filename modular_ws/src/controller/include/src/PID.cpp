#include "PID.hpp"

static float e_roll, e_pitch, e_eski_roll, e_eski_pitch, ie_roll, ie_pitch; //PID hatalari
static float imax=120, imin=-120;
static const int f = 40;
static const float st = 1/(float)f;

static float ie_roll_sat, ie_pitch_sat;
static float pd_roll_buf, pd_pitch_buf;
static float pd_roll_sat_buf, pd_pitch_sat_buf;


double P_Angle(double alpha_des, double alpha, double Kp_angle) {
	double P;
	double e = alpha_des - alpha;
	P = Kp_angle*e;
    return P;

}


double PD_Rate_Roll(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd) {
	double P, I, D, pd,de;
	e_eski_roll = e_roll;
	e_roll = alpha_dot_des - alpha_dot;
  double e_roll_int = e_roll;

  if((int)pd_roll_buf != (int)pd_roll_sat_buf) {
    if(sgn(e_roll) == sgn(pd_roll_sat_buf)) {
      e_roll_int = 0;
    }
  }


	de = e_roll - e_eski_roll;
  ie_roll = ie_roll + e_roll_int;
  ie_roll_sat = ie_roll;
	

	P = Kp*e_roll; D = Kd*de; I = Ki * ie_roll_sat;

	pd = P + I + D;
  	pd_roll_buf = pd;
	pd  = Sat(pd,  300, -300);
	pd_roll_sat_buf = pd;
    return pd;

}

double PD_Rate_Pitch(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd) {
	double P, I, D, pd,de;
	e_eski_pitch = e_pitch;
	e_pitch = alpha_dot_des - alpha_dot;
  double e_pitch_int = e_pitch;
  
    if((int)pd_pitch_buf != (int)pd_pitch_sat_buf) {
    if(sgn(e_pitch) == sgn(pd_pitch_sat_buf)) {
      e_pitch_int = 0;
    }
  }


	de = e_pitch - e_eski_pitch;
  ie_pitch = ie_pitch + e_pitch_int;
  ie_pitch_sat = ie_pitch;

      
	P = Kp*e_pitch; D = Kd*de; I = Ki * ie_pitch_sat;

  

	pd = P + I + D;
  	pd_pitch_buf = pd;
    pd = Sat(pd, 300, -300);
	pd_pitch_sat_buf = pd;

    return pd;

}

double P_Rate_Yaw(double alpha_dot_des, double alpha_dot, double Kp) {
	double P;
	double e_yaw = alpha_dot_des - alpha_dot;	
	P = Kp*e_yaw;
	P    = Sat(P,    300, -300);

    return P;

}

double sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

 double Sat(double pwm, int max, int min) {
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

float pwm2ang(unsigned short int pwm) {
	int in_min  = 1000;
	int in_max  = 2000;
	int out_min = -30;
	int out_max  = 30;

	return (pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Convert pwm to motor speed for simulation
float pwm2mot(unsigned short int pwm, int dir) {
	float in_min  = 1000;
	float in_max  = 2000;
	float out_min = 0;
	float out_max  = 1326;

	return (float)(dir) * ((float)pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
