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
#define USE_SIM 1

//Sim includes
#if USE_SIM
  #include "ros/ros.h"
  #include "std_msgs/String.h"
  #include "controller/attitude.h"
  #include "nav_msgs/Odometry.h"
  #include "mav_msgs/Actuators.h"
  #include "sensor_msgs/Imu.h"
  #include <tf/tf.h>
  #include "gazebo_msgs/ModelStates.h"
  #include "gazebo_msgs/LinkState.h"
  #include <vector>  
#endif


#include "PID.h"
#include "Kalman.h"
#include "Controller.h"
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




/* USER CODE END PM */


/* USER CODE BEGIN PV */
//SIM
#if USE_SIM
  nav_msgs::Odometry sensor_data;
  sensor_msgs::Imu imu_data;
  mav_msgs::Actuators motors;
  controller::attitude attitude;
#endif

float gyroX, gyroY, gyroZ, gyro_e_x, gyroX_a,gyroX_a_x, accX, accY, accZ;
float gyroa_x, gyroa_y, gyroa_z;
float roll_des, pitch_des, yaw_des;
double roll_rate_des, pitch_rate_des, yaw_rate_des;
const float rad2deg = 180/3.14;

const int f = 40;
const float st = 1/(float)f;
float w1, w2, w3, w4; //Motor hizlari
float pwm_trim = 1550;
float gyro[3];
float acc[3];

struct state {
    float angles[3];
    float rates[3];
    float bias[3];
};

struct controller_output {
    float w[4];
};
struct state state;
//SIM
unsigned int start;



int timer;
unsigned short int IC_val1, IC_val2, pwm_input;
unsigned short int pwm1, pwm2;
unsigned short int pwm_mid = 1200;
char buf[32];

#if USE_SIM
  ros::Publisher att_pub;

  void IMUCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  //sensor_data.pose = sensor_msg->pose;
  //sensor_data.twist = sensor_msg->twist;
  imu_data.angular_velocity    = imu_msg->angular_velocity;
  imu_data.linear_acceleration = imu_msg->linear_acceleration;

  gyroX =    imu_data.angular_velocity.x*rad2deg;
  gyroY = -1*imu_data.angular_velocity.y*rad2deg;
  gyroZ =    imu_data.angular_velocity.z*rad2deg;
  gyro[0] = gyroX;
  gyro[1] = gyroY;
  gyro[2] = gyroZ;


  accX = imu_data.linear_acceleration.x;
  accY = imu_data.linear_acceleration.y;
  accZ = imu_data.linear_acceleration.z;
  acc[0] = accX;
  acc[1] = accY;
  acc[2] = accZ;

  state = Kalman_Filtresi(gyro, acc);


  //printf("\nsensor: %.2f",sensor_data.pose[1].position.x);
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
    printf("\nroll: %.2f",roll);
    printf("\npitch: %.2f",pitch);
    printf("\nyaw: %.2f",yaw);
    

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
    printf("\nroll_rate: %.2f",roll_rate);
    printf("\npitch_rate: %.2f",pitch_rate);
    printf("\nyaw_rate: %.2f",yaw_rate);
    */

    //==============
}
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MPU6050_Baslat(void);
void PWMYaz(unsigned short int pwm1, unsigned short int pwm2);
void MotorBaslat(void);
bool while_condition(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  */
int main(int argc, char **argv) {
  
  /* USER CODE BEGIN 1 */
  #if USE_SIM 
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("iris/imu", 1000, IMUCallback);
    ros::Subscriber odom_sub = n.subscribe("iris/ground_truth/odometry", 1000, realCallBack);
    ros::Publisher motor_pub = n.advertise<mav_msgs::Actuators>("iris/command/motor_speed", 100);
    att_pub   = n.advertise<controller::attitude>("controller/attitude", 100);
    ros::Rate loop_rate(f);
  #endif

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
  while (while_condition())
  
  {
      

	 // HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    roll_des = 0;
    pitch_des = 10;
    yaw_rate_des = 0;
    struct controller_output controller_output = Controller(state, roll_des, pitch_des, yaw_rate_des, gyro, acc);

 
    printf("\nw1: %.2f", controller_output.w[0]);
    printf("\nw2: %.2f", controller_output.w[1]);
    printf("\nw3: %.2f", controller_output.w[2]);
    printf("\nw4: %.2f", controller_output.w[3]); 

    float roll =  state.angles[0];
    float pitch = state.angles[1];
    float yaw =   state.angles[2];

  
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;

    attitude.roll_des = roll_des;
    attitude.pitch_des = pitch_des;
    attitude.yaw_des = yaw_des;
  


    attitude.roll_rate_des = roll_rate_des;
    attitude.pitch_rate_des = pitch_rate_des;
    attitude.yaw_rate_des = yaw_rate_des;




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

bool while_condition(void) {
  #if USE_SIM
    return ros::ok();
  #else
    return true;
  #endif
}


