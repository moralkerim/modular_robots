#include "Controller.hpp"



Controller::Controller() {}

void Controller::Run (void) {
        //printf("\ngyroX: %.2f",gyro[0]);
        //printf("\naccX: %.2f",acc[0]);
		vel_controller_counter++;

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
        
    if(!swarm) {
    	  //bench settings
//        Kp_roll = 0.2; //0.3
//        Ki_roll = 0.1;  //0.008
//        Kd_roll = 0.05; //0.015

         Kp_roll = 0.22; //0.3
         Ki_roll = 0.08;  //0.008
         Kd_roll = 0.02; //0.015

         Kp_pitch = Kp_roll;	//0.8
         Ki_pitch = Ki_roll;
         Kd_pitch = Kd_roll;

         Kp_yaw = 5.0;// 1;
         Ki_yaw = 5;// 1;
    }

    else {


        Kp_roll = 0.9; //0.3
        Ki_roll = 0.00;  //0.008
        Kd_roll = 0.03; //0.007 0.01

        Kp_pitch = Kp_roll;	//0.8
        Ki_pitch = Ki_roll;
        Kd_pitch = Kd_roll;

        Kp_yaw = 10.0;// 1;
        Ki_yaw = 0;// 1;
    }

    int thr;
    int thr2 = pid_roll.Sat(ch3, 2000, 1000);

    switch(mod) {
    	case STABILIZE:
    	{
    	    thr = pid_roll.Sat(ch3, 1800, 1000);
            roll_des     = state_des.angles[0];
            pitch_des 	 = state_des.angles[1];
            yaw_rate_des = state_des.rates[2];

            roll_des 	  	= roll_des_filt.Run(roll_des);
            pitch_des 		= pitch_des_filt.Run(pitch_des);
            yaw_rate_des  	= yaw_des_filt.Run(yaw_rate_des);
    	    break;
    	}

    	case ALT_HOLD:
    	{
    		F = p_alt.PI_Vel(z0, z, z_vel, Kp_alt, Ki_alt, ch3) + m*g;
			float deg2rad = 0.0175;
			float roll_r = roll * deg2rad;
			float pitch_r = pitch * deg2rad;
			float b2e = 1 / cos(roll_r) / cos(pitch_r);

			F = F * b2e ; // Body to Earth
			F = p_alt.Sat(F, F_max, F_min);
			thr = p_alt.F2thr(F);
			thr = p_alt.Sat(thr, 1800, 1100);
			alt_thr = thr;
			z0 = p_alt.zi;

            roll_des     = state_des.angles[0];
            pitch_des 	 = state_des.angles[1];
            yaw_rate_des = state_des.rates[2];

            roll_des 	  	= roll_des_filt.Run(roll_des);
            pitch_des 		= pitch_des_filt.Run(pitch_des);
            yaw_rate_des  	= yaw_des_filt.Run(yaw_rate_des);
			break;
    	}

    	case LOITER:
    	{
    	    thr = pid_roll.Sat(ch3, 1800, 1000);

    		//roll_des  = p_velx.PI_Vel(0, y, vy, Kp_vel, Ki_vel, ch1);

    	    if(vel_controller_counter >= 10) {
    	    	//PID_Pos(float pos_des, float pos, float Kp, float Ki, float Kd)
    	    	vel_controller_counter = 0;
        		pitch_des = p_velx.PID_Pos(x0,x,Kp_velx,Ki_velx,Kd_velx);
        		roll_des  = p_vely.PID_Pos(y0,y,Kp_vely,Ki_vely,Kd_vely);

        		pitch_des = p_velx.Sat(pitch_des, 20, -20);
        		roll_des = p_vely.Sat(roll_des, 20, -20);

    	    }

            yaw_rate_des = state_des.rates[2];
            yaw_rate_des  	= yaw_des_filt.Run(yaw_rate_des);

    		break;
    	}

    }

    //if(angle_loop_counter == 3) {
    	//float sqrt_controller(float alpha_des, float _alpha_des, uint8_t angle_counter,float Kff);
    	angle_ff_roll  = pid_roll.sqrt_controller(roll_des, _roll_des, angle_loop_counter,Kff);
    	angle_ff_pitch = pid_pitch.sqrt_controller(pitch_des, _pitch_des, angle_loop_counter,Kff);
    	roll_rate_des = pid_roll.P_Angle(roll_des,roll, Kp_angle,Ki_angle) + angle_ff_roll;
    	pitch_rate_des = pid_pitch.P_Angle(pitch_des,pitch, Kp_angle,Ki_angle) + angle_ff_pitch ;
    	_roll_des  = roll_des;
    	_pitch_des = pitch_des;
    	angle_loop_counter = 0;
    //}



//	float pd_roll_ff  = pid_roll.RateFF(roll_rate_des);
//	float pd_pitch_ff = pid_roll.RateFF(pitch_rate_des);

	pd_roll  = pid_roll.PID_Rate2(roll_rate_des,roll_rate, roll, Kp_roll, Ki_roll, Kd_roll, Kp_angle);// + pd_roll_ff;
	pd_pitch = pid_pitch.PID_Rate2(pitch_rate_des,pitch_rate, pitch, Kp_pitch,Ki_pitch,Kd_pitch, Kp_angle);// + pd_pitch_ff;
	p_yaw    = pid_yaw.PD_Rate(yaw_rate_des,yaw_rate,Kp_yaw,Ki_yaw,0);
	angle_loop_counter++;

#ifdef UAV1


    int pwm1 = thr + pd_pitch - pd_roll  - p_yaw + PITCH_TRIM - ROLL_TRIM - YAW_TRIM;
    int pwm2 = thr - pd_pitch + pd_roll  - p_yaw - PITCH_TRIM + ROLL_TRIM - YAW_TRIM;
    int pwm3 = thr + pd_pitch + pd_roll  + p_yaw + PITCH_TRIM + ROLL_TRIM + YAW_TRIM;
    int pwm4 = thr - pd_pitch - pd_roll  + p_yaw - PITCH_TRIM - ROLL_TRIM + YAW_TRIM;

//    //Test bench
//    int pwm1 = thr - pd_roll  - ROLL_TRIM;
//    int pwm2 = thr + pd_roll  + ROLL_TRIM;
//    int pwm3 = thr + pd_roll  + ROLL_TRIM;
//    int pwm4 = thr - pd_roll  - ROLL_TRIM;


#endif

#ifdef UAV2


    int pwm1 = thr + pd_pitch - pd_roll  - p_yaw + PITCH_TRIM - ROLL_TRIM;
    int pwm2 = thr - pd_pitch + pd_roll  - p_yaw - PITCH_TRIM + ROLL_TRIM;
    int pwm3 = thr + pd_pitch + pd_roll  + p_yaw + PITCH_TRIM + ROLL_TRIM;
    int pwm4 = thr - pd_pitch - pd_roll  + p_yaw - PITCH_TRIM - ROLL_TRIM;

#endif

    //Saturate pwm values
    pwm1 = (int)pid_roll.Sat(pwm1,PWM_UPPER,PWM_LOWER,thr);
    pwm2 = (int)pid_roll.Sat(pwm2,PWM_UPPER,PWM_LOWER,thr);
    pwm3 = (int)pid_roll.Sat(pwm3,PWM_UPPER,PWM_LOWER,thr);
    pwm4 = (int)pid_roll.Sat(pwm4,PWM_UPPER,PWM_LOWER,thr);

    //SECOND DRONE
    /*
    controller_output_pwm2[0] = thr + pd_pitch - pd_roll  + p_yaw;
    controller_output_pwm2[1] = thr - pd_pitch + pd_roll  + p_yaw;
    controller_output_pwm2[2] = thr + pd_pitch + pd_roll  - p_yaw;
    controller_output_pwm2[3] = thr - pd_pitch - pd_roll  - p_yaw;
    */

    if(swarm) {
        controller_output_pwm2[0] = pwm1;
        controller_output_pwm2[1] = pwm2;
        controller_output_pwm2[2] = pwm3;
        controller_output_pwm2[3] = pwm4;
    }

    else {
        controller_output_pwm2[0] = 1000;
        controller_output_pwm2[1] = 1000;
        controller_output_pwm2[2] = 1000;
        controller_output_pwm2[3] = 1000;
    }



    //Saturate pwm values
    controller_output_pwm2[0] = (int)pid_roll.Sat(controller_output_pwm2[0],PWM_UPPER,1000,thr);
    controller_output_pwm2[1] = (int)pid_roll.Sat(controller_output_pwm2[1],PWM_UPPER,1000,thr);
    controller_output_pwm2[2] = (int)pid_roll.Sat(controller_output_pwm2[2],PWM_UPPER,1000,thr);
    controller_output_pwm2[3] = (int)pid_roll.Sat(controller_output_pwm2[3],PWM_UPPER,1000,thr);

    // MOTOR TEST
/*
    pwm1 = 1000;
    pwm2 = 1000;
    pwm3 = 1000;
    pwm4 = 1200;
*/
    //Convert pwm to motor speed 
    /*
    w1 = pid_roll.pwm2mot(pwm1, 1);
    w2 = pid_roll.pwm2mot(pwm2, 1);
    w3 = pid_roll.pwm2mot(pwm3,-1);
    w4 = pid_roll.pwm2mot(pwm4,-1);
*/

   // std::vector<float> controller_output = 	{w1,w2,w3,w4};
    controller_output_pwm[0] = pwm1;
    controller_output_pwm[1] = pwm2;
    controller_output_pwm[2] = pwm3;
    controller_output_pwm[3] = pwm4;

  //  return controller_output;
}

Controller::~Controller() {}
