class PID {

    private:
        float e_roll, e_pitch, e_eski_roll, e_eski_pitch, ie_roll, ie_pitch; //PID hatalari
        float imax=120, imin=-120;
        const int f = 40;
        const float st = 1/(float)f;

        float ie_roll_sat, ie_pitch_sat;
        float pd_roll_buf, pd_pitch_buf;
        float pd_roll_sat_buf, pd_pitch_sat_buf;

    public:
        PID();
        double P_Angle(double alpha_des, double alpha, double Kp_angle);
        double PD_Rate_Roll(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd);
        double PD_Rate_Pitch(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd);
        double P_Rate_Yaw(double alpha_dot_des, double alpha_dot, double Kp);
        double Sat(double pwm, int max, int min);
        float pwm2ang(unsigned short int pwm);
        float pwm2mot(unsigned short int pwm, int dir);
        double sgn(double v);
        ~PID();
};