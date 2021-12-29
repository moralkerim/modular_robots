#include "Controller.hpp"


struct state {
    float angles[3];
    float rates[3];
    float bias[3];
};

Controller::Controller() {}

std::vector<double> Controller::Run (struct state state, struct state state_des) {
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

    float roll_rate_des = pid.P_Angle(roll_des,roll, Kp_angle);
    float pitch_rate_des = pid.P_Angle(pitch_des,pitch, Kp_angle);
/*
    //printf("\nroll_rate_des: %.2f",roll_rate_des);
    //printf("\npitch_rate_des: %.2f",pitch_rate_des);
    //printf("\nyaw_rate_des: %.2f",yaw_rate_des);
*/  //printf("\nroll_rate_des: %.2f",roll_rate_des);
    double pd_roll  = pid.PD_Rate_Roll(roll_rate_des,roll_rate, Kp_roll, Ki_roll, Kd_roll);
    //printf("\npitch_rate_des: %.2f",pitch_rate_des);
    double pd_pitch = pid.PD_Rate_Pitch(pitch_rate_des,pitch_rate,Kp_pitch,Ki_pitch,Kd_pitch);
    double p_yaw    = pid.P_Rate_Yaw(yaw_rate_des,yaw_rate,Kp_yaw);


    //printf("\npd_roll: %.2f",pd_roll);
    //printf("\npd_pitch: %.2f",pd_pitch);
    //printf("\np_yaw: %.2f",p_yaw);

    ////printf("\nst: %.3f",st);

    unsigned int pwm1 = pwm_trim + pd_pitch - pd_roll  - p_yaw;
    unsigned int pwm2 = pwm_trim - pd_pitch + pd_roll  - p_yaw;
    unsigned int pwm3 = pwm_trim + pd_pitch + pd_roll  + p_yaw;
    unsigned int pwm4 = pwm_trim - pd_pitch - pd_roll  + p_yaw;

    //Saturate pwm values
    pwm1 = (int)pid.Sat(pwm1,PWM_UPPER,PWM_LOWER); 
    pwm2 = (int)pid.Sat(pwm2,PWM_UPPER,PWM_LOWER); 
    pwm3 = (int)pid.Sat(pwm3,PWM_UPPER,PWM_LOWER); 
    pwm4 = (int)pid.Sat(pwm4,PWM_UPPER,PWM_LOWER);

    //Convert pwm to motor speed 
    w1 = pid.pwm2mot(pwm1, 1);
    w2 = pid.pwm2mot(pwm2, 1);
    w3 = pid.pwm2mot(pwm3,-1);
    w4 = pid.pwm2mot(pwm4,-1);

    std::vector<double> controller_output = 	{w1,w2,w3,w4};
    controller_output_pwm = {pwm1,pwm2,pwm3,pwm4};
    return controller_output;
}

Controller::~Controller() {}
