#include "Controller.h"

#define PWM_UPPER 2000
#define PWM_LOWER 1050

static float pitch_bias, roll_bias, yaw_bias;
static double roll, pitch, yaw;
static double roll_rate, pitch_rate, yaw_rate;

static float w1, w2, w3, w4; //Motor hizlari
static float pwm_trim = 1550;
static const float rad2deg = 180/3.14;

static const int f = 40;
static const float st = 1/(float)f;
//PID Katsayilari
static double Kp_pitch = 1.5;
static double Ki_pitch = 0.3;
static double Kd_pitch = 0.05*f;

static double Kp_roll = 0.5;
static double Ki_roll = 0.05;
static double Kd_roll = 0.05*f;

static double Kp_yaw = 0.1;

static float Kp_angle = 0.03*f;

struct state {
    float angles[3];
    float rates[3];
    float bias[3];
};

struct controller_output {
    float w[4];
};

struct controller_output Controller (struct state state, float roll_des, float pitch_des, float yaw_rate_des,float gyro[3], float acc[3]) {
        printf("\ngyroX: %.2f",gyro[0]);
        printf("\naccX: %.2f",acc[0]);
        
        roll  = state.angles[0];
        pitch = state.angles[1];
        yaw    = state.angles[2];
        printf("\nroll: %.2f",roll);
        printf("\npitch: %.2f",pitch);

        roll_rate  = state.rates[0];
        pitch_rate = state.rates[1];
        yaw_rate   = state.rates[2];

        roll_bias = state.bias[0];
        pitch_bias = state.bias[1];
        yaw_bias = state.bias[2];
        
    float roll_rate_des = P_Angle(roll_des,roll, Kp_angle);
    float pitch_rate_des = P_Angle(pitch_des,pitch, Kp_angle);
/*
    printf("\nroll_rate_des: %.2f",roll_rate_des);
    printf("\npitch_rate_des: %.2f",pitch_rate_des);
    printf("\nyaw_rate_des: %.2f",yaw_rate_des);
*/  printf("\nroll_rate_des: %.2f",roll_rate_des);
    double pd_roll  = PD_Rate_Roll(roll_rate_des,roll_rate, Kp_roll, Ki_roll, Kd_roll);
    printf("\npitch_rate_des: %.2f",pitch_rate_des);
    double pd_pitch = PD_Rate_Pitch(pitch_rate_des,pitch_rate,Kp_pitch,Ki_pitch,Kd_pitch);
    double p_yaw    = P_Rate_Yaw(yaw_rate_des,yaw_rate,Kp_yaw);


    printf("\npd_roll: %.2f",pd_roll);
    printf("\npd_pitch: %.2f",pd_pitch);
    printf("\np_yaw: %.2f",p_yaw);

    //printf("\nst: %.3f",st);

    unsigned int pwm1 = pwm_trim + pd_pitch - pd_roll  - p_yaw;
    unsigned int pwm2 = pwm_trim - pd_pitch + pd_roll  - p_yaw;
    unsigned int pwm3 = pwm_trim + pd_pitch + pd_roll  + p_yaw;
    unsigned int pwm4 = pwm_trim - pd_pitch - pd_roll  + p_yaw;

    //Saturate pwm values
    pwm1 = (int)Sat(pwm1,PWM_UPPER,PWM_LOWER); 
    pwm2 = (int)Sat(pwm2,PWM_UPPER,PWM_LOWER); 
    pwm3 = (int)Sat(pwm3,PWM_UPPER,PWM_LOWER); 
    pwm4 = (int)Sat(pwm4,PWM_UPPER,PWM_LOWER);

    //Convert pwm to motor speed 
    w1 = pwm2mot(pwm1, 1);
    w2 = pwm2mot(pwm2, 1);
    w3 = pwm2mot(pwm3,-1);
    w4 = pwm2mot(pwm4,-1);


    struct controller_output controller_output;
    controller_output.w[0] = w1;
    controller_output.w[1] = w2;
    controller_output.w[2] = w3;
    controller_output.w[3] = w4;

    return controller_output;
}
