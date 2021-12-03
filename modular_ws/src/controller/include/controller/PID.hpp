double P_Angle(double alpha_des, double alpha, double Kp_angle);
double PD_Rate_Roll(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd);
double PD_Rate_Pitch(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd);
double P_Rate_Yaw(double alpha_dot_des, double alpha_dot, double Kp);
double Sat(double pwm, int max, int min);
float pwm2ang(unsigned short int pwm);
float pwm2mot(unsigned short int pwm, int dir);
double sgn(double v);