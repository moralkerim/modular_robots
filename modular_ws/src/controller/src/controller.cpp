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
#include "sensor_msgs/Imu.h"
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
sensor_msgs::Imu imu_data;
mav_msgs::Actuators motors;
controller::attitude attitude;

float gyroX, gyroY, gyroZ, gyro_e_x, gyroX_a,gyroX_a_x, accX, accY, accZ;
float pitch_acc, roll_acc, yaw_acc;
float gyroa_x, gyroa_y, gyroa_z;
float alpha, alpha_dot, pitch_bias, roll_bias, yaw_bias;
double alpha_des, alpha_dot_des;
double roll, pitch, yaw;
double roll_des, pitch_des, yaw_des;
double roll_rate, pitch_rate, yaw_rate;
double roll_rate_des, pitch_rate_des, yaw_rate_des;
float w1, w2, w3, w4; //Motor hizlari
float pwm_trim = 1550;
const float rad2deg = 180/3.14;
float S11_m_pitch, S12_m_pitch, S21_m_pitch, S22_m_pitch;
float S11_p_pitch, S12_p_pitch, S21_p_pitch, S22_p_pitch;
float Kt11_pitch, Kt21_pitch;
double sa = 1.15e-7; double sb = 1.15e-7;

float S11_m_roll, S12_m_roll, S21_m_roll, S22_m_roll;
float S11_p_roll, S12_p_roll, S21_p_roll, S22_p_roll;
float Kt11_roll, Kt21_roll;

float S11_m_yaw, S12_m_yaw, S21_m_yaw, S22_m_yaw;
float S11_p_yaw, S12_p_yaw, S21_p_yaw, S22_p_yaw;
float Kt11_yaw, Kt21_yaw;

double Q = 1.6e-5; //0.5 -- onceki deger.
float e_roll, e_pitch, e_eski_roll, e_eski_pitch, ie_roll, ie_pitch; //PID hatalari
float imax=120, imin=-120;

//SIM
unsigned int start;


const int f = 40;
const float st = 1/(float)f;
//PID Katsayilari
double Kp_pitch = 1.5;
double Ki_pitch = 0.3;
double Kd_pitch = 0.05*f;

double Kp_roll = 0.5;
double Ki_roll = 0.05;
double Kd_roll = 0.05*f;

double Kp_yaw = 0.1;

float Kp_angle = 0.03*f;
float pd_roll_buf, pd_pitch_buf;
float pd_roll_sat_buf, pd_pitch_sat_buf;
float ie_roll_sat, ie_pitch_sat;

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
double P_Rate_Yaw(double alpha_dot_des, double alpha_dot, double Kp);
double Sat(double pwm, int max, int min);
float pwm2ang(unsigned short int pwm);
void MotorBaslat(void);
void Kontrolcu(void);
void sensorCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
void realCallBack(const nav_msgs::Odometry::ConstPtr& sensor_msg);
float pwm2mot(unsigned short int pwm, int dir);
double sgn(double v);

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
  ros::Subscriber sub = n.subscribe("iris/imu", 1000, sensorCallback);
  ros::Subscriber odom_sub = n.subscribe("iris/ground_truth/odometry", 1000, realCallBack);
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
    double p_yaw    = P_Rate_Yaw(yaw_rate_des,yaw_rate,Kp_yaw);

    pd_roll  = Sat(pd_roll,  300, -300);
    pd_pitch = Sat(pd_pitch, 300, -300);
    p_yaw    = Sat(p_yaw,    300, -300);

    pd_roll_sat_buf = pd_roll;
    pd_pitch_sat_buf = pd_pitch;


    ROS_INFO("pd_roll: %.2f",pd_roll);
    ROS_INFO("pd_pitch: %.2f",pd_pitch);
    ROS_INFO("p_yaw: %.2f",p_yaw);

    //ROS_INFO("st: %.3f",st);

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
    
    att_pub.publish(attitude);
    start++;
    loop_rate.sleep();
    ros::spinOnce();

  }
      /*
      std::vector<double> motor_stop {0,0,0,0};
      motors.angular_velocities = motor_stop;
      motor_pub.publish(motors);
      */
      return 0;
  /* USER CODE END 3 */
}

void sensorCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  //sensor_data.pose = sensor_msg->pose;
  //sensor_data.twist = sensor_msg->twist;
  imu_data.angular_velocity    = imu_msg->angular_velocity;
  imu_data.linear_acceleration = imu_msg->linear_acceleration;

  gyroX =    imu_data.angular_velocity.x*rad2deg;
  gyroY = -1*imu_data.angular_velocity.y*rad2deg;
  gyroZ =    imu_data.angular_velocity.z*rad2deg;

  accX = imu_data.linear_acceleration.x;
  accY = imu_data.linear_acceleration.y;
  accZ = imu_data.linear_acceleration.z;
  Kalman_Filtresi();
  //ROS_INFO("sensor: %.2f",sensor_data.pose[1].position.x);
}

void Kalman_Filtresi(void) {

    //---IMU KİSMİ----
    //=================================
  float acctop=sqrt(accX*accX+accY*accY+accZ*accZ);
  double st = 0.003803;
  pitch_acc =  asin(accX/acctop)*rad2deg;
  roll_acc  =  asin(accY/acctop)*rad2deg;
  yaw_acc   =  asin(accZ/acctop)*rad2deg;
  attitude.roll_acc = roll_acc;
  attitude.pitch_acc = pitch_acc;
  attitude.yaw_acc = yaw_acc;
  //ROS_INFO("pitc_acc: %.2f", pitch_acc);
    
  //Pitch angle
	//**Tahmin**
	pitch = pitch - pitch_bias*st + gyroY*st;
	S11_m_pitch = 2*sa+st*st*sb; S12_m_pitch=-st*sb;
	S21_m_pitch = -st*sb; 	   S22_m_pitch=2*sb;

	//**Düzeltme**
	Kt11_pitch = S11_m_pitch / (S11_m_pitch+Q);
	Kt21_pitch = S21_m_pitch / (S21_m_pitch+Q);

	pitch = pitch - Kt11_pitch*(pitch-pitch_acc);
	pitch_bias = pitch_bias - Kt21_pitch*(pitch-pitch_acc);

	S11_p_pitch = -S11_m_pitch*(Kt11_pitch-1);  S12_p_pitch = -S12_m_pitch*(Kt11_pitch-1);
	S21_p_pitch = S21_m_pitch-S11_m_pitch*Kt21_pitch; S22_p_pitch = S22_m_pitch-S12_m_pitch*Kt21_pitch;

	S11_m_pitch = S11_p_pitch; S12_m_pitch = S12_p_pitch; S21_m_pitch = S21_p_pitch; S22_m_pitch = S22_p_pitch; 

  pitch_rate = gyroY;
    //=================================

  //Roll angle
	//**Tahmin**
	roll = roll - roll_bias*st + gyroX*st;
	S11_m_roll = 2*sa+st*st*sb; S12_m_pitch=-st*sb;
	S21_m_roll = -st*sb; 	   S22_m_pitch=2*sb;

	//**Düzeltme**
	Kt11_roll = S11_m_roll / (S11_m_roll+Q);
	Kt21_roll = S21_m_roll / (S21_m_roll+Q);

	roll = roll - Kt11_roll*(roll-roll_acc);
	roll_bias = roll_bias - Kt21_roll*(roll-roll_acc);

	S11_p_roll = -S11_m_roll*(Kt11_roll-1);  S12_p_roll = -S12_m_roll*(Kt11_roll-1);
	S21_p_roll = S21_m_roll-S11_m_roll*Kt21_roll; S22_p_roll = S22_m_roll-S12_m_roll*Kt21_roll;

	S11_m_roll = S11_p_roll; S12_m_roll = S12_p_roll; S21_m_roll = S21_p_roll; S22_m_roll = S22_p_roll; 
  roll_rate = gyroX;
    //=================================

   //Yaw angle
	//**Tahmin**
	yaw = yaw - yaw_bias*st + gyroZ*st;
	S11_m_yaw = 2*sa+st*st*sb; S12_m_pitch=-st*sb;
	S21_m_yaw = -st*sb; 	   S22_m_pitch=2*sb;

	//**Düzeltme**
	Kt11_yaw = S11_m_yaw / (S11_m_yaw+Q);
	Kt21_yaw = S21_m_yaw / (S21_m_yaw+Q);

	yaw = yaw - Kt11_yaw*(yaw-yaw_acc);
	yaw_bias = yaw_bias - Kt21_yaw*(yaw-yaw_acc);

	S11_p_yaw = -S11_m_yaw*(Kt11_yaw-1);  S12_p_yaw = -S12_m_yaw*(Kt11_yaw-1);
	S21_p_yaw = S21_m_yaw-S11_m_yaw*Kt21_yaw; S22_p_yaw = S22_m_yaw-S12_m_yaw*Kt21_yaw;

	S11_m_yaw = S11_p_yaw; S12_m_yaw = S12_p_yaw; S21_m_yaw = S21_p_yaw; S22_m_yaw = S22_p_yaw; 
  yaw_rate = gyroZ;
    //=================================

  attitude.roll = roll;
  attitude.pitch = pitch;
  attitude.yaw = yaw;

}

void realCallBack(const nav_msgs::Odometry::ConstPtr& sensor_msg) {
  sensor_data.pose = sensor_msg->pose;
  sensor_data.twist = sensor_msg->twist;

      //--SIMULASYON--
    //==============
    
    tf::Quaternion q(sensor_data.pose.pose.orientation.x,
                     sensor_data.pose.pose.orientation.y,
                     sensor_data.pose.pose.orientation.z,
                     sensor_data.pose.pose.orientation.w); 
    tf::Matrix3x3 m(q);
    double roll_real, pitch_real, yaw_real;
    m.getRPY(roll_real, pitch_real, yaw_real,1);
    roll_real  =      rad2deg*roll_real;
    pitch_real =   -1*rad2deg*pitch_real;
    yaw_real   =      rad2deg*yaw_real;
    
 

    /*
    ROS_INFO("roll: %.2f",roll);
    ROS_INFO("pitch: %.2f",pitch);
    ROS_INFO("yaw: %.2f",yaw);
    

    roll_rate = sensor_data.twist.twist.angular.x;
    pitch_rate = sensor_data.twist.twist.angular.y;
    yaw_rate = sensor_data.twist.twist.angular.z;

    roll_rate  = rad2deg*roll_rate;
    pitch_rate = -1*rad2deg*pitch_rate;
    yaw_rate   = rad2deg*yaw_rate;
    */

    attitude.roll_real = roll_real;
    attitude.pitch_real = pitch_real;
    attitude.yaw_real = yaw_real;

    
    //attitude.roll_rate  = roll_rate;
    //attitude.pitch_rate = pitch_rate;
    //attitude.yaw_rate   = yaw_rate;
    

    

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
    ROS_INFO("alpha_des: %.2f",alpha_des);
    ROS_INFO("alpha: %.2f",alpha);
    ROS_INFO("P OUT: %.2f",P);
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

  ROS_INFO("alpha_dot_des: %.2f",alpha_dot_des);
  ROS_INFO("alpha_dot: %.2f",alpha_dot);
  ROS_INFO("Rate error: %.2f",e_roll);
	de = e_roll - e_eski_roll;
  ROS_INFO("de error:     %.2f",de);
  ROS_INFO("e error:      %.2f",e_roll);
  ROS_INFO("e_eski error: %.2f",e_eski_pitch);
  ie_roll = ie_roll + e_roll_int;
  ie_roll_sat = ie_roll;

	ROS_INFO("ie_roll: %.2f",ie_roll);		

	P = Kp*e_roll; D = Kd*de; I = Ki * ie_roll_sat;

	pd = P + I + D;
  pd_roll_buf = pd;
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

  ROS_INFO("alpha_dot_des: %.2f",alpha_dot_des);
  ROS_INFO("alpha_dot: %.2f",alpha_dot);
  ROS_INFO("Rate error: %.2f",e_pitch);
	de = e_pitch - e_eski_pitch;
  ROS_INFO("de error:     %.2f",de);
  ROS_INFO("e error:      %.2f",e_pitch);
  ROS_INFO("e_eski error: %.2f",e_eski_pitch);
  ie_pitch = ie_pitch + e_pitch_int;
  ie_pitch_sat = ie_pitch;

      
  ROS_INFO("ie_pitch_sat: %.2f",ie_pitch_sat);	
	P = Kp*e_pitch; D = Kd*de; I = Ki * ie_pitch_sat;

  

	pd = P + I + D;
  pd_pitch_buf = pd;
    return pd;

}

double P_Rate_Yaw(double alpha_dot_des, double alpha_dot, double Kp) {
	double P;
	double e_yaw = alpha_dot_des - alpha_dot;	
	P = Kp*e_yaw;
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

