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
#include <tf/tf.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkState.h"
#include <math.h>
#include "string.h"
#include <stdio.h>


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
gazebo_msgs::ModelStates sensor_data;
gazebo_msgs::LinkState motors;

float gyroX, gyroY, gyroZ, gyro_e_x, gyroX_a,gyroX_a_x, accX, accY, accZ;
float pitch_acc;
float gyroa_x, gyroa_y, gyroa_z;
float alpha, alpha_dot, bias;
float alpha_des, alpha_dot_des;
double roll, pitch, yaw;
double roll_des, pitch_des, yaw_des;
double roll_rate, pitch_rate, yaw_rate;
double roll_rate_des, pitch_rate_des, yaw_rate_des;
float w1, w2, w3, w4; //Motor hizlari
const float pwm_trim = 1800;
const float rad2deg = 180/3.14;
float S11_m, S12_m, S21_m, S22_m;
float S11_p, S12_p, S21_p, S22_p;
float Kt11, Kt21;
float sa = 0.001; float sb = 0.001;
float Q = 1; //0.5 -- onceki deger.
float e, e_eski; //PID hatalari


const unsigned int f = 40;
const float st = 1/(float)f;
//PD Katsayilari
float Kp = 1.2;
float Kd =  1.5*f;

float Kp_angle = 0.03;
int timer;
unsigned short int IC_val1, IC_val2, pwm_input;
unsigned short int pwm1, pwm2;
unsigned short int pwm_mid = 1200;
char buf[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MPU6050_Baslat(void);
void Kalman_Filtresi(void);
void PWMYaz(unsigned short int pwm1, unsigned short int pwm2);
double P_Angle(double alpha_des, double alpha);
double PD_Rate(double alpha_dot_des, double alpha_dot);
int Sat(int pwm, int max, int min);
float pwm2ang(unsigned short int pwm);
void MotorBaslat(void);
void Kontrolcu(void);
void sensorCallback(const gazebo_msgs::ModelStates::ConstPtr& sensor_msg);
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
  ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, sensorCallback);
  ros::Publisher motor_pub = n.advertise<gazebo_msgs::LinkState>("gazebo/set_link_state", 1000);
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

    roll_des = 30;
    pitch_des = 0;
    roll_rate_des = P_Angle(roll_des,roll);
    pitch_rate_des = P_Angle(pitch_des,pitch);
    yaw_rate_des = P_Angle(yaw_des,yaw);
/*
    ROS_INFO("roll_rate_des: %.2f",roll_rate_des);
    ROS_INFO("pitch_rate_des: %.2f",pitch_rate_des);
    ROS_INFO("yaw_rate_des: %.2f",yaw_rate_des);
*/
    double pd_roll  = PD_Rate(roll_rate_des,roll_rate);
    double pd_pitch = PD_Rate(pitch_rate_des,pitch_rate);
    double pd_yaw   = PD_Rate(yaw_rate_des,yaw_rate);

    pd_roll  = Sat(pd_roll,  400, -400);
    pd_pitch = Sat(pd_pitch, 400, -400);
    pd_yaw   = Sat(pd_yaw,   400, -400);

    ROS_INFO("pd_roll: %.2f",pd_roll);
    ROS_INFO("pd_pitch: %.2f",pd_pitch);
    ROS_INFO("pd_yaw: %.2f",pd_yaw);

    ROS_INFO("st: %.3f",st);

    unsigned int pwm1 = pwm_trim - pd_roll + pd_pitch;// + pd_yaw;
    unsigned int pwm2 = pwm_trim + pd_roll - pd_pitch;// + pd_yaw;
    unsigned int pwm3 = pwm_trim + pd_roll + pd_pitch;// - pd_yaw;
    unsigned int pwm4 = pwm_trim - pd_roll - pd_pitch;// - pd_yaw;

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

    //motor1
    motors.link_name = "iris::iris_demo::iris::rotor_0";
    motors.twist.angular.z = w1;
    motors.reference_frame = "iris::iris_demo::iris::rotor_0";
    motor_pub.publish(motors);
    //motor2
    motors.link_name = "iris::iris_demo::iris::rotor_1";
    motors.twist.angular.z = w2;
    motors.reference_frame = "iris::iris_demo::iris::rotor_1";
    motor_pub.publish(motors);
    //motor3
    motors.link_name = "iris::iris_demo::iris::rotor_2";
    motors.reference_frame = "iris::iris_demo::iris::rotor_2";
    motors.twist.angular.z = w3;
    motor_pub.publish(motors);
    //motor4
    motors.link_name = "iris::iris_demo::iris::rotor_3";
    motors.reference_frame = "iris::iris_demo::iris::rotor_3";
    motors.twist.angular.z = w4;
    motor_pub.publish(motors);

    loop_rate.sleep();
    ros::spinOnce();

  }
      return 0;
  /* USER CODE END 3 */
}

void sensorCallback(const gazebo_msgs::ModelStates::ConstPtr& sensor_msg)
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
    tf::Quaternion q(sensor_data.pose[1].orientation.x,
                     sensor_data.pose[1].orientation.y,
                     sensor_data.pose[1].orientation.z,
                     sensor_data.pose[1].orientation.w); 
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    roll  =   rad2deg*roll;
    pitch =   -1*rad2deg*pitch;
    yaw   =   rad2deg*yaw;
    
    ROS_INFO("roll: %.2f",roll);
    ROS_INFO("pitch: %.2f",pitch);
    ROS_INFO("yaw: %.2f",yaw);
    

    roll_rate = sensor_data.twist[1].angular.x;
    pitch_rate = sensor_data.twist[1].angular.y;
    yaw_rate = sensor_data.twist[1].angular.z;

    roll_rate  = rad2deg*roll_rate;
    pitch_rate = rad2deg*pitch_rate;
    yaw_rate   = rad2deg*yaw_rate;

    ROS_INFO("roll_rate: %.2f",roll_rate);
    ROS_INFO("pitch_rate: %.2f",pitch_rate);
    ROS_INFO("yaw_rate: %.2f",yaw_rate);

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


double PD_Rate(double alpha_dot_des, double alpha_dot) {
	double P, D, pd,de;
	e_eski = e;
	e = alpha_dot_des - alpha_dot;
  ROS_INFO("alpha_dot_des: %.2f",alpha_dot_des);
  ROS_INFO("alpha_dot: %.2f",alpha_dot);
  ROS_INFO("Rate error: %.2f",e);
	de = e - e_eski;
	P = Kp*e; D = Kd*de;
	pd = P + D;
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
	float out_max  = 1300;

	return (float)(dir) * ((float)pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

