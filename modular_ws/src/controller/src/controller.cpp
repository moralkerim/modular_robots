/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//Sim includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controller/attitude.h"
#include "nav_msgs/Odometry.h"
#include "mav_msgs/Actuators.h"
#include <tf/tf.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkState.h"
#include <math.h>
#include "string.h"
#include <stdio.h>
#include <vector>  

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


#define PWM_UPPER 2000
#define PWM_LOWER 1050

/* USER CODE END PM */


/* USER CODE BEGIN PV */
//SIM
nav_msgs::Odometry sensor_data;
mav_msgs::Actuators motors;
controller::attitude attitude;

float gyroX, gyroY, gyroZ, gyro_e_x, gyroX_a,gyroX_a_x, accX, accY, accZ;
float pitch_acc;
float gyroa_x, gyroa_y, gyroa_z;
float alpha, alpha_dot, bias;
double alpha_des, alpha_dot_des;
double roll, pitch, yaw;
double roll_des, pitch_des, yaw_des;
double roll_rate, pitch_rate, yaw_rate;
double roll_rate_des, pitch_rate_des, yaw_rate_des;
float w1, w2, w3, w4; //Motor hizlari
float pwm_trim = 1550;
const float rad2deg = 180/3.14;
float S11_m, S12_m, S21_m, S22_m;
float S11_p, S12_p, S21_p, S22_p;
float Kt11, Kt21;
float sa = 0.001; float sb = 0.001;
float Q = 1; //0.5 -- onceki deger.
float e_roll, e_pitch, e_eski_roll, e_eski_pitch, ie_roll, ie_pitch; //PID hatalari
float imax=120, imin=-120;

//SIM
unsigned int start;


const int f = 40;
const float st = 1/(float)f;
//PID Katsayilari
double Kp_pitch = 1.2;
double Ki_pitch = 0.00;
double Kd_pitch = 0.000*f;

double Kp_roll = 0.7;
double Ki_roll = 0.00;
double Kd_roll = 0.0*f;

float Kp_angle = 0.03*f;
int timer;
unsigned short int IC_val1, IC_val2, pwm_input;
unsigned short int pwm1, pwm2;
unsigned short int pwm_mid = 1200;
char buf[32];

ros::Publisher att_pub;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MPU6050_Baslat(void);
void Kalman_Filtresi(void);
void PWMYaz(unsigned short int pwm1, unsigned short int pwm2);
double P_Angle(double alpha_des, double alpha);
double PD_Rate_Roll(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd);
double PD_Rate_Pitch(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd);
int Sat(int pwm, int max, int min);
float pwm2ang(unsigned short int pwm);
void MotorBaslat(void);
void Kontrolcu(void);
void sensorCallback(const nav_msgs::Odometry::ConstPtr& sensor_msg);
float pwm2mot(unsigned short int pwm, int dir);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  */
int main(int argc, char **argv) {
  /* USER CODE BEGIN 1 */
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("iris/ground_truth/odometry", 100, sensorCallback);
  ros::Publisher motor_pub = n.advertise<mav_msgs::Actuators>("iris/command/motor_speed", 100);
  att_pub   = n.advertise<controller::attitude>("controller/attitude", 100);
  ros::Rate loop_rate(f);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
 
  /* USER CODE BEGIN 2 */

  //Gyro kalibrasyon hatalarını hesapla.

  //Kontrolcü Timer'ı

  //PWM çıkış timer'ları
  

  //PWM Input Capture Kanalları

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (ros::ok())
  {
      

	 // HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    roll_des = 0;
    pitch_des = 10;
    yaw_rate_des = 0;
    roll_rate_des = P_Angle(roll_des,roll);
    pitch_rate_des = P_Angle(pitch_des,pitch);

    attitude.roll_des = roll_des;
    attitude.pitch_des = pitch_des;
    attitude.yaw_des = yaw_des;


    attitude.roll_rate_des = roll_rate_des;
    attitude.pitch_rate_des = pitch_rate_des;
    attitude.yaw_rate_des = yaw_rate_des;
/*
    ROS_INFO("roll_rate_des: %.2f",roll_rate_des);
    ROS_INFO("pitch_rate_des: %.2f",pitch_rate_des);
    ROS_INFO("yaw_rate_des: %.2f",yaw_rate_des);
*/  ROS_INFO("roll_rate_des: %.2f",roll_rate_des);
    double pd_roll  = PD_Rate_Roll(roll_rate_des,roll_rate, Kp_roll, Ki_roll, Kd_roll);
    ROS_INFO("pitch_rate_des: %.2f",pitch_rate_des);
    double pd_pitch = PD_Rate_Pitch(pitch_rate_des,pitch_rate,Kp_pitch,Ki_pitch,Kd_pitch);
    //double pd_yaw   = PD_Rate_Pitch(yaw_rate_des,yaw_rate,Kp_pitch);

    pd_roll  = Sat(pd_roll,  300, -300);
    pd_pitch = Sat(pd_pitch, 300, -300);
    //pd_yaw   = Sat(pd_yaw,   300, -300);

    ROS_INFO("pd_roll: %.2f",pd_roll);
    ROS_INFO("pd_pitch: %.2f",pd_pitch);
    //ROS_INFO("pd_yaw: %.2f",pd_yaw);

    //ROS_INFO("st: %.3f",st);

    unsigned int pwm1 = pwm_trim + pd_pitch - pd_roll;//  - pd_yaw;
    unsigned int pwm2 = pwm_trim - pd_pitch + pd_roll;//  - pd_yaw;
    unsigned int pwm3 = pwm_trim + pd_pitch + pd_roll;//  + pd_yaw;
    unsigned int pwm4 = pwm_trim - pd_pitch - pd_roll;//  + pd_yaw;

    //Saturate pwm values
    pwm1 = Sat(pwm1,PWM_UPPER,PWM_LOWER); pwm2 = Sat(pwm2,PWM_UPPER,PWM_LOWER); pwm3 = Sat(pwm3,PWM_UPPER,PWM_LOWER); pwm4 = Sat(pwm4,PWM_UPPER,PWM_LOWER);

    //Convert pwm to motor speed 
    w1 = pwm2mot(pwm1, 1);
    w2 = pwm2mot(pwm2, 1);
    w3 = pwm2mot(pwm3,-1);
    w4 = pwm2mot(pwm4,-1);

    ROS_INFO("w1: %.2f", w1);
    ROS_INFO("w2: %.2f", w2);
    ROS_INFO("w3: %.2f", w3);
    ROS_INFO("w4: %.2f", w4);

    std::vector<double> motor_speeds {abs(w1),abs(w2),abs(w3),abs(w4)};
    motors.angular_velocities = motor_speeds;
    motor_pub.publish(motors);

    
    if(start > 100) {
      pwm_trim = 1500;
    }
    
    
    loop_rate.sleep();
    ros::spinOnce();
    att_pub.publish(attitude);
    start++;

  }
      /*
      std::vector<double> motor_stop {0,0,0,0};
      motors.angular_velocities = motor_stop;
      motor_pub.publish(motors);
      */
      return 0;
  /* USER CODE END 3 */
}

void sensorCallback(const nav_msgs::Odometry::ConstPtr& sensor_msg)
{
  sensor_data.pose = sensor_msg->pose;
  sensor_data.twist = sensor_msg->twist;
  Kalman_Filtresi();
  //ROS_INFO("sensor: %.2f",sensor_data.pose[1].position.x);
}

void Kalman_Filtresi(void) {

    //---IMU KİSMİ----
    //=================================
    /*

	//**Tahmin**
	alpha = alpha - bias*st + gyroX*st;
	S11_m = 2*sa+st*st*sb; S12_m=-st*sb;
	S21_m = -st*sb; 	   S22_m=2*sb;

	//**Düzeltme**
	Kt11 = S11_m / (S11_m+Q);
	Kt21 = S21_m / (S21_m+Q);

	alpha = alpha - Kt11*(alpha-pitch_acc);
	bias = bias - Kt21*(alpha-pitch_acc);

	S11_p = -S11_m*(Kt11-1);  S12_p = -S12_m*(Kt11-1);
	S21_p = S21_m-S11_m*Kt21; S22_p = S22_m-S12_m*Kt21;

	S11_m = S11_p; S12_m = S12_p; S21_m = S21_p; S22_m = S22_p; */
    //=================================

    //--SIMULASYON--
    //==============
    tf::Quaternion q(sensor_data.pose.pose.orientation.x,
                     sensor_data.pose.pose.orientation.y,
                     sensor_data.pose.pose.orientation.z,
                     sensor_data.pose.pose.orientation.w); 
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw,1);
    roll  =   rad2deg*roll;
    pitch =   -1*rad2deg*pitch;
    yaw   =   rad2deg*yaw;
    
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;

    /*
    ROS_INFO("roll: %.2f",roll);
    ROS_INFO("pitch: %.2f",pitch);
    ROS_INFO("yaw: %.2f",yaw);
    */

    roll_rate = sensor_data.twist.twist.angular.x;
    pitch_rate = sensor_data.twist.twist.angular.y;
    yaw_rate = sensor_data.twist.twist.angular.z;

    roll_rate  = rad2deg*roll_rate;
    pitch_rate = -1*rad2deg*pitch_rate;
    yaw_rate   = rad2deg*yaw_rate;

    attitude.roll_rate  = roll_rate;
    attitude.pitch_rate = pitch_rate;
    attitude.yaw_rate   = yaw_rate;

    

   /*
    ROS_INFO("roll_rate: %.2f",roll_rate);
    ROS_INFO("pitch_rate: %.2f",pitch_rate);
    ROS_INFO("yaw_rate: %.2f",yaw_rate);
    */

    //==============
}

void PWMYaz(unsigned short int pwm1, unsigned short int pwm2) {

	  
}

double P_Angle(double alpha_des, double alpha) {
	double P;
	double e = alpha_des - alpha;
	P = Kp_angle*e;
    //ROS_INFO("P OUT: %.2f",P);
    return P;

}


double PD_Rate_Roll(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd) {
	double P, I, D, pd,de;
	e_eski_roll = e_roll;
	e_roll = alpha_dot_des - alpha_dot;
  ROS_INFO("alpha_dot_des: %.2f",alpha_dot_des);
  ROS_INFO("alpha_dot: %.2f",alpha_dot);
  ROS_INFO("Rate error: %.2f",e_roll);
	de = e_roll - e_eski_roll;
  ROS_INFO("de error:     %.2f",de);
  ROS_INFO("e error:      %.2f",e_roll);
  ROS_INFO("e_eski error: %.2f",e_eski_pitch);
  ie_roll = ie_roll + e_roll;
			if (ie_roll>imax)
			{
				ie_roll=imax;
			}
			
			else if (ie_roll<imin)
			{
				ie_roll=imin;						//I kontrolcu icin doyma blogu
			}

    /*
      if(abs(e_roll) < 1) {
        ie_roll = 0;
      }
      */

	ROS_INFO("ie_roll: %.2f",ie_roll);		

	P = Kp*e_roll; D = Kd*de; I = Ki * ie_roll;

	pd = P + I + D;
    return pd;

}

double PD_Rate_Pitch(double alpha_dot_des, double alpha_dot, double Kp, double Ki, double Kd) {
	double P, I, D, pd,de;
	e_eski_pitch = e_pitch;
	e_pitch = alpha_dot_des - alpha_dot;
  ROS_INFO("alpha_dot_des: %.2f",alpha_dot_des);
  ROS_INFO("alpha_dot: %.2f",alpha_dot);
  ROS_INFO("Rate error: %.2f",e_pitch);
	de = e_pitch - e_eski_pitch;
  ROS_INFO("de error:     %.2f",de);
  ROS_INFO("e error:      %.2f",e_pitch);
  ROS_INFO("e_eski error: %.2f",e_eski_pitch);
  ie_pitch = ie_pitch + e_pitch;
			if (ie_pitch>imax)
			{
				ie_pitch=imax;
			}
			
			else if (ie_pitch<imin)
			{
				ie_pitch=imin;						//I kontrolcu icin doyma blogu
			}
      
			if(abs(e_pitch) < 1) {
        ie_pitch = 0;
      }
      
  ROS_INFO("ie_pitch: %.2f",ie_pitch);	
	P = Kp*e_pitch; D = Kd*de; I = Ki * ie_pitch;

  

	pd = P + I + D;
    return pd;

}

 int Sat(int pwm, int max, int min) {
	int pwm_out;
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
	int16_t in_min  = 1000;
	int16_t in_max  = 2000;
	int16_t out_min = -30;
	int16_t out_max  = 30;

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

