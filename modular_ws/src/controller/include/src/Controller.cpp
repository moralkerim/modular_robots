#include "Controller.hpp"


struct state {
    float angles[3];
    float rates[3];
    float bias[3];
};

Controller::Controller() {}

std::vector<double> Controller::Run (struct state state, struct state state_des, int thr) {
        //printf("\ngyroX: %.2f",gyro[0]);
        //printf("\naccX: %.2f",acc[0]);
        
        roll  = state.angles[0];
        pitch = state.angles[1];
        yaw    = state.angles[2];
        //printf("\nroll: %.2f",roll);
        //printf("\npitch: %.2f",pitch);

        roll_rate  = state.rates[0];
        pitch_rate = state.rates[1];
        yaw_rate   = state.rates[2];

        roll_bias = state.bias[0];
        pitch_bias = state.bias[1];
        yaw_bias = state.bias[2];
        
        float roll_des     = state_des.angles[0];
        float pitch_des 	 = state_des.angles[1];
        float yaw_rate_des = state_des.rates[2];

        roll_des 	  	= roll_des_filt.Run(roll_des);
        pitch_des 		= pitch_des_filt.Run(pitch_des);
        yaw_rate_des  	= yaw_des_filt.Run(yaw_rate_des);

        /*
    roll_rate_des  = pid_roll.P_Sqrt(roll_des,  roll,  Kp_angle);
    pitch_rate_des = pid_roll.P_Sqrt(pitch_des, pitch, Kp_angle);
*/


    roll_rate_des = pid_roll.P_Angle(roll_des,roll, Kp_angle);
    pitch_rate_des = pid_pitch.P_Angle(pitch_des,pitch, Kp_angle);


/*
    //printf("\nroll_rate_des: %.2f",roll_rate_des);
    //printf("\npitch_rate_des: %.2f",pitch_rate_des);
    //printf("\nyaw_rate_des: %.2f",yaw_rate_des);
*/  //printf("\nroll_rate_des: %.2f",roll_rate_des);
    /*
    pd_roll  = pid_roll.PD_Rate(roll_rate_des,roll_rate, Kp_roll, Ki_roll, Kd_roll);
    //printf("\npitch_rate_des: %.2f",pitch_rate_des);
    pd_pitch = pid_pitch.PD_Rate(pitch_rate_des,pitch_rate,Kp_pitch,Ki_pitch,Kd_pitch);
    //p_yaw    = pid_yaw.P_Rate_Yaw(yaw_rate_des,yaw_rate,Kp_yaw);
    p_yaw    = pid_yaw.PD_Rate(yaw_rate_des,yaw_rate,Kp_yaw,Ki_yaw,0);
*/
    pd_roll  = pid_roll.PID_Rate2(roll_rate_des,roll_rate, Kp_roll, Ki_roll, Kd_roll);
    pd_pitch = pid_pitch.PID_Rate2(pitch_rate_des,pitch_rate,Kp_pitch,Ki_pitch,Kd_pitch);
    p_yaw    = pid_yaw.PD_Rate(yaw_rate_des,yaw_rate,Kp_yaw,Ki_yaw,0);


    //printf("\npd_roll: %.2f",pd_roll);
    //printf("\npd_pitch: %.2f",pd_pitch);
    //printf("\np_yaw: %.2f",p_yaw);

    ////printf("\nst: %.3f",st);

    thr = pid_roll.Sat(thr, 1800, 1000);

    int pwm1 = thr + pd_pitch - pd_roll  - p_yaw + PITCH_TRIM - ROLL_TRIM;
    int pwm2 = thr - pd_pitch + pd_roll  - p_yaw - PITCH_TRIM + ROLL_TRIM;
    int pwm3 = thr + pd_pitch + pd_roll  + p_yaw + PITCH_TRIM + ROLL_TRIM;
    int pwm4 = thr - pd_pitch - pd_roll  + p_yaw - PITCH_TRIM - ROLL_TRIM;


    //Saturate pwm values
    pwm1 = (int)pid_roll.Sat(pwm1,PWM_UPPER,PWM_LOWER,thr);
    pwm2 = (int)pid_roll.Sat(pwm2,PWM_UPPER,PWM_LOWER,thr);
    pwm3 = (int)pid_roll.Sat(pwm3,PWM_UPPER,PWM_LOWER,thr);
    pwm4 = (int)pid_roll.Sat(pwm4,PWM_UPPER,PWM_LOWER,thr);

    //Convert pwm to motor speed 
    w1 = pid_roll.pwm2mot(pwm1, 1);
    w2 = pid_roll.pwm2mot(pwm2, 1);
    w3 = pid_roll.pwm2mot(pwm3,-1);
    w4 = pid_roll.pwm2mot(pwm4,-1);


    std::vector<double> controller_output = 	{w1,w2,w3,w4};
    controller_output_pwm[0] = pwm1;
    controller_output_pwm[1] = pwm2;
    controller_output_pwm[2] = pwm3;
    controller_output_pwm[3] = pwm4;

    return controller_output;
}

std::vector<double> Controller::Run (struct state state, struct state state_des, float z_vel, float z0, float z, float ch3) {
        //printf("\ngyroX: %.2f",gyro[0]);
        //printf("\naccX: %.2f",acc[0]);

        roll  = state.angles[0];
        pitch = state.angles[1];
        yaw    = state.angles[2];
        //printf("\nroll: %.2f",roll);
        //printf("\npitch: %.2f",pitch);

        roll_rate  = state.rates[0];
        pitch_rate = state.rates[1];
        yaw_rate   = state.rates[2];

        roll_bias = state.bias[0];
        pitch_bias = state.bias[1];
        yaw_bias = state.bias[2];

        float roll_des     = state_des.angles[0];
        float pitch_des 	 = state_des.angles[1];
        float yaw_rate_des = state_des.rates[2];

        roll_des 	  	= roll_des_filt.Run(roll_des);
        pitch_des 		= pitch_des_filt.Run(pitch_des);
        yaw_rate_des  	= yaw_des_filt.Run(yaw_rate_des);


    roll_rate_des = pid_roll.P_Angle(roll_des,roll, Kp_angle);
    pitch_rate_des = pid_pitch.P_Angle(pitch_des,pitch, Kp_angle);


/*
    //printf("\nroll_rate_des: %.2f",roll_rate_des);
    //printf("\npitch_rate_des: %.2f",pitch_rate_des);
    //printf("\nyaw_rate_des: %.2f",yaw_rate_des);
*/  //printf("\nroll_rate_des: %.2f",roll_rate_des);

    /*
    pd_roll  = pid_roll.PD_Rate(roll_rate_des,roll_rate, Kp_roll, Ki_roll, Kd_roll);
    //printf("\npitch_rate_des: %.2f",pitch_rate_des);
    pd_pitch = pid_pitch.PD_Rate(pitch_rate_des,pitch_rate,Kp_pitch,Ki_pitch,Kd_pitch);
    //p_yaw    = pid_yaw.P_Rate_Yaw(yaw_rate_des,yaw_rate,Kp_yaw);
    p_yaw    = pid_yaw.PD_Rate(yaw_rate_des,yaw_rate,Kp_yaw,Ki_yaw,0);
	*/

    pd_roll  = pid_roll.PID_Rate2(roll_rate_des,roll_rate, Kp_roll, Ki_roll, Kd_roll);
    pd_pitch = pid_pitch.PID_Rate2(pitch_rate_des,pitch_rate,Kp_pitch,Ki_pitch,Kd_pitch);
    p_yaw    = pid_yaw.PD_Rate(yaw_rate_des,yaw_rate,Kp_yaw,Ki_yaw,0);


    //printf("\npd_roll: %.2f",pd_roll);
    //printf("\npd_pitch: %.2f",pd_pitch);
    //printf("\np_yaw: %.2f",p_yaw);

    ////printf("\nst: %.3f",st);
    //F = p_alt.PD_Rate(0, z_vel, Kp_alt, Ki_alt, 0) + m*g;
    //double PI_Alt(double z0, double z, double v, double Kp_alt, double Ki_alt, unsigned int ch3);
    F = p_alt.PI_Alt(z0, z, z_vel, Kp_alt, Ki_alt, ch3) + m*g;
    float deg2rad = 0.0175;
    float roll_r = roll * deg2rad;
    float pitch_r = pitch * deg2rad;
    float b2e = 1 / cos(roll_r) / cos(pitch_r);

    F = F * b2e ; // Body to Earth
    F = p_alt.Sat(F, F_max, F_min);
    float thr = p_alt.F2thr(F);
    thr = p_alt.Sat(thr, 1800, 1100);
    alt_thr = thr;
    int pwm1 = thr + pd_pitch - pd_roll  - p_yaw + PITCH_TRIM - ROLL_TRIM;
    int pwm2 = thr - pd_pitch + pd_roll  - p_yaw - PITCH_TRIM + ROLL_TRIM;
    int pwm3 = thr + pd_pitch + pd_roll  + p_yaw + PITCH_TRIM + ROLL_TRIM;
    int pwm4 = thr - pd_pitch - pd_roll  + p_yaw - PITCH_TRIM - ROLL_TRIM;


    //Saturate pwm values
    pwm1 = (int)pid_roll.Sat(pwm1,PWM_UPPER,PWM_LOWER,thr);
    pwm2 = (int)pid_roll.Sat(pwm2,PWM_UPPER,PWM_LOWER,thr);
    pwm3 = (int)pid_roll.Sat(pwm3,PWM_UPPER,PWM_LOWER,thr);
    pwm4 = (int)pid_roll.Sat(pwm4,PWM_UPPER,PWM_LOWER,thr);

    //Convert pwm to motor speed
    w1 = pid_roll.pwm2mot(pwm1, 1);
    w2 = pid_roll.pwm2mot(pwm2, 1);
    w3 = pid_roll.pwm2mot(pwm3,-1);
    w4 = pid_roll.pwm2mot(pwm4,-1);


    std::vector<double> controller_output = 	{w1,w2,w3,w4};
    controller_output_pwm[0] = pwm1;
    controller_output_pwm[1] = pwm2;
    controller_output_pwm[2] = pwm3;
    controller_output_pwm[3] = pwm4;

    return controller_output;
}

Controller::~Controller() {}
