/* USER CODE BEGIN Header */
/**
  *********************  *********************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/detail/twist__struct.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>

#include "usart.h"
#include "tim.h"

//#include "math.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

typedef struct{
	uint16_t state, current;
	int16_t rpm;
	int position, position_p, encoder, encoder_p;
} motor_packet;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/*
char Run_State = 'B';
char Motor_State = 'T';
char Run_Dir = 'F';
*/

#define _USE_MATH_DEFINES


#define DATA_SIZE 24

//uint8_t Command_Request_Data[20]= {0xb7,0xb8,0x01,0x0a,0x01,0x05,0x80};
//uint8_t Command_Request_Data[7]= {0xB7,0xB8,0x01,0x0A,0x01,0x05,0x80};
//uint8_t Command_Request_Data[7]= {0xB7,0xB8,0x01,0x0A,0x01,0x3D,0x48};
uint8_t Command_Request_Data[7]= {0xB7,0xB8,0x01,0x04,0x01,0xD2,0xB9};

uint8_t Command_Response1[50]="01234567890ABCDEF";//dummy
uint8_t Command_Response2[DATA_SIZE]="01234567890ABCDEF";//dummy
//uint8_t Command_Response2[23]="01234567890ABCDEF";//dummy
//uint8_t Command_Response2[25]="01234567890ABCDEF";//dummy


motor_packet motor1, motor2;



char Run_State;
char Motor_State;
char Run_Dir;


// ?��?��?��?? 각속?��?�� 모든 값을 �??��?�� ???��
float linear_velocity_x;
float linear_velocity_y;
float linear_velocity_z;

float angular_velocity_x;
float angular_velocity_y;
float angular_velocity_z;

int m1_hall=0, m2_hall=0;

int m1_rev=0, m2_rev=0;
int m1_deg=0, m2_deg=0;
int m1_deg_10=0, m2_deg_10=0;
int m1_deg_10_p=0, m2_deg_10_p=0;
int m1_deg_1_10=0, m2_deg_1_10=0;
int m1_rpm=0, m2_rpm=0;
int m1_rpm_p=0, m2_rpm_p=0;
int m1_rpm_p_10=0, m2_rpm_p_10=0;

//std_msgs__msg__Int32 msg;
int hall1 = 0, hall2 = 0;

typedef struct{
	double vx, vy, vz;
} Twist_value;
/*
Twist_value Linear = {0, 0, 0};
Twist_value Angular = {0, 0, 0};*/
Twist_value Linear;
Twist_value Angular;


//int count_sec = 0;
//int turn_flag = 0;
int m_mode = 0; // 0: stop, 1: straight, 2: rotation mode
double pwm_p = 0;
double pwm_m = 0;

double angle;

int timeout = 10000000;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void messageCallback(const void *msgin);
void messageCallback_test1(const void *msgin);
void messageCallback_test2(const void *msgin);
void twist_subscription_callback(const void * msgin);


//my motor test functions

//int MOTION_MODE(double x, double y, double z);

void RUN_DIR(char);

void RUN_RB(char);
void MOTOR_SS(char);

double MOTOR_CAL_LINE_PWM(double,double,double);

void PWM_L(double);
void PWM_R(double);

double CAL_LinearSpeed(double x, double y, double z);
double CAL_AngularSpeed(double x, double y, double z);

//void TURN(double);

double TURN(double ang);
//int MOTION_MODE(double, double, double);
//void MOTION_MODE(double, double, double);
void MOTION_MODE(double lx, double ly, double lz, double ax, double ay, double az);


int16_t hex_to_signed_int(const char *hex_str);





/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__); 
  }


	int length;
	motor1.position_p = 0;
	motor2.position_p = 0;






  // micro-ROS app

  rcl_publisher_t publisher;
  std_msgs__msg__Int32 msg;
//test
  rcl_publisher_t publisher_test;
  std_msgs__msg__Int32_test1 msg_test;

  rcl_subscription_t subscriber_test1;
  std_msgs__msg__Int32_test2 submsg_test1;

  rcl_subscription_t subscriber_test2;
  std_msgs__msg__Int32_test3 submsg_test2;
//
  rcl_publisher_t publisher_1;
  std_msgs__msg__Int32_pub1 pubmsg_1;

  rcl_publisher_t publisher_2;
  std_msgs__msg__Int32_pub2 pubmsg_2;


  rcl_publisher_t publisher_3;
  std_msgs__msg__Int32_pub1 pubmsg_3;


  rcl_subscription_t subscriber_2;
//  std_msgs__msg__String_sub1 submsg_1;
  geometry_msgs__msg__Twist submsg_2;
//  geometry_msgs__msg__Twist * submsg_2_1;

  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);



  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // create publisher example
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cubemx_publisher");


  // create publisher_2
/*  rclc_publisher_init_default(
    &publisher_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "r_enc");

  	  // r_wheel_vel
  // create publisher_3
  rclc_publisher_init_default(
    &publisher_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "l_enc");
*/

  // create publisher_3
  rclc_publisher_init_default(
    &publisher_3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "enc");


  	  //l_enc
  // create publisher_test
  rclc_publisher_init_default(
    &publisher_test,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cubemx_publisher_test");

  // create subscriber_test1
  rclc_subscription_init_default(
      &subscriber_2,
      &node,
//      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, twist),
      // ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      // "diffbot_base_controller/cmd_vel_unstamped");
  	  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  	  "cmd_vel");
  // create subscriber_test2
/*  rclc_subscription_init_default(
      &subscriber_test2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//	    "cubemx_publisher_test");
        "r_wheel_vel");*/

  // create subscriber
/*  rclc_subscription_init_default(
      &subscriber_1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "r_wheel_vel_String");*/




  // add subscriber to executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
//  rclc_executor_add_subscription(&executor, &subscriber_1, &submsg_1, &messageCallback, ON_NEW_DATA);

//  rclc_executor_add_subscription(&executor, &subscriber_test1, &submsg_test1, &messageCallback_test1, ON_NEW_DATA);
//  rclc_executor_add_subscription(&executor, &subscriber_1, &submsg_test2, &twist_subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_2, &submsg_2, &twist_subscription_callback, ON_NEW_DATA);

  msg.data = 0;
  pubmsg_1.hall_1 = 0;
  pubmsg_2.hall_2 = 0;


//  RUN_INIT();


  for(;;)
  {
    // publish message


      // hall pulse count
	  if(motor1.encoder > 500 && motor1.encoder < -500){
		pubmsg_1.hall_1 = (int32_t)motor1.encoder_p;

	  }
	  else{
		pubmsg_1.hall_1 = (int32_t)motor1.encoder;
		motor1.encoder_p = motor1.encoder;
	  }

	  if(motor2.encoder > 500 && motor2.encoder < -500){
		pubmsg_2.hall_2 = (int32_t)motor2.encoder_p;

	  }
	  else{
		pubmsg_2.hall_2 = (int32_t)motor2.encoder;
		motor2.encoder_p = motor2.encoder;
	  }

	  pubmsg_3.hall_1 = pubmsg_1.hall_1 + 10000*pubmsg_2.hall_2;

/*		 rcl_ret_t ret_3 = rcl_publish(&publisher_2, &pubmsg_2, NULL);
		 rcl_ret_t ret_2 = rcl_publish(&publisher_1, &pubmsg_1, NULL);*/

		 rcl_ret_t ret_4 = rcl_publish(&publisher_3, &pubmsg_3, NULL);


/*	   if (ret_2 != RCL_RET_OK)
	   {
		 printf("Error publishing (line %d)\n", __LINE__);
	   }
	   if (ret_3 != RCL_RET_OK)
	   {
		 printf("Error publishing (line %d)\n", __LINE__);
	   }*/

//    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
/*	 rcl_ret_t ret_3 = rcl_publish(&publisher_2, &pubmsg_2, NULL);
	 rcl_ret_t ret_2 = rcl_publish(&publisher_1, &pubmsg_1, NULL);*/
//    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    rcl_ret_t ret_test = rcl_publish(&publisher_test, &msg_test, NULL);
/*    if (ret != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__); 
    }*/
    /*
    if (ret_2 != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }
    if (ret_3 != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }*/


	  // Transmit command to request data
	  length = sizeof(Command_Request_Data);
	  HAL_UART_Transmit(&huart5, Command_Request_Data, length, 10);


    length = DATA_SIZE;
//    if (HAL_UART_Receive(&huart5, Command_Response1, 50, 100) == HAL_OK) {

    	HAL_UART_Receive(&huart5, Command_Response1, 50, 500);

  	  for(int i = 0; i < 50-DATA_SIZE; i++){
  		  if(Command_Response1[i] == 0xB8){
//    	    	  if(Command_Response1[i] == 0xB8 && Command_Response1[i+1] == 0xB7 && Command_Response1[i+2] == 0x01 && Command_Response1[i+3] == 0xC1 && Command_Response1[i+4] == 0x11){

    	    	  if(Command_Response1[i] == 0xB8 && Command_Response1[i+1] == 0xB7 && Command_Response1[i+2] == 0x01){

  	              // Copy received data to another buffer (for example purposes)
  	    		  for (int j = 0; j < DATA_SIZE; j++) {
  	    		          Command_Response2[j] = Command_Response1[i];
  	    		          i++;
					  }


  	    		  motor1.rpm = Command_Response2[5] | Command_Response2[6]<<8;
  	    		  motor1.current = Command_Response2[7] | Command_Response2[8]<<8;
  	    		  motor1.state = Command_Response2[9];
  	    		  motor1.position = Command_Response2[10] | Command_Response2[11]<<8 | Command_Response2[12]<<16 | Command_Response2[13]<<24;
  	    		  motor1.encoder = motor1.position - motor1.position_p;

  	    		  motor2.rpm = Command_Response2[14] | Command_Response2[15]<<8;
  	    		  motor2.current = Command_Response2[16] | Command_Response2[17]<<8;
  	    		  motor2.state = Command_Response2[18];
  	    		  motor2.position = Command_Response2[19] | Command_Response2[20]<<8 | Command_Response2[21]<<16 | Command_Response2[22]<<24;
//  	    		  motor2.encoder = motor2.position - motor2.position_p;
  	    		  motor2.encoder = -(motor2.position - motor2.position_p);


      			  break;
      	    	  }
      		  }
      	  }
//        }
/*    else{
    	motor1.encoder = -1;
    	motor2.encoder = -1;
    }*/


    memset(Command_Response1, 0, sizeof(Command_Response1)); // 버퍼 초기화
    memset(Command_Response2, 0, sizeof(Command_Response2)); // 버퍼 초기화











    // subscribe message
//    rclc_executor_spin(&executor);
//    rclc_executor_spin_some(&executor, timeout);
//    rclc_executor_spin_one_period(&executor, timeout);

    /*      // hall pulse count
      	pubmsg_1.hall_1 = hall1;
      	pubmsg_2.hall_2 = hall2;

          // reset the hall sensor data
      	hall1 = 0;
      	hall2 = 0;*/

        // hall pulse count
/*    if(motor1.encoder > 500 && motor1.encoder < -500){
    	pubmsg_1.hall_1 = (int32_t)motor1.encoder_p;

    }
    else{
    	pubmsg_1.hall_1 = (int32_t)motor1.encoder;
    	motor1.encoder_p = motor1.encoder;
    }

    if(motor2.encoder > 500 && motor2.encoder < -500){
    	pubmsg_2.hall_2 = (int32_t)motor2.encoder_p;

    }
    else{
    	pubmsg_2.hall_2 = (int32_t)motor2.encoder;
    	motor2.encoder_p = motor2.encoder;
    }


   	 rcl_ret_t ret_3 = rcl_publish(&publisher_2, &pubmsg_2, NULL);
   	 rcl_ret_t ret_2 = rcl_publish(&publisher_1, &pubmsg_1, NULL);


     if (ret_2 != RCL_RET_OK)
     {
       printf("Error publishing (line %d)\n", __LINE__);
     }
     if (ret_3 != RCL_RET_OK)
     {
       printf("Error publishing (line %d)\n", __LINE__);
     }*/
        // reset the hall sensor data
/*    	hall1 = 0;
    	hall2 = 0;*/

/*
	Linear.vx = submsg_2_1->linear.x;
	Linear.vy = submsg_2_1->linear.y;
	Linear.vz = submsg_2_1->linear.z;*/

/*	Linear.vx = submsg_2.linear.x;
	Linear.vy = submsg_2.linear.y;
	Linear.vz = submsg_2.linear.z;*/

	// move the robot.
//	MOTION_MODE(Linear.vx, Linear.vy, Linear.vz);
/*
	if(m_mode == 0){
		// robot stop
		Run_State = 'B';
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, SET);	// Motor1 RUN/BRK - BRK
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, SET);	// Motor2 RUN/BRK - BRK
	}
	else if(m_mode == 1){
		// robot move straight
		if(Linear.vx >0){
			Run_Dir = 'F';
			Run_State = 'U';
			PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
			PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
		}
		else if (Linear.vx < 0)
		{
			Run_Dir = 'O';
			Run_State = 'U';
			PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
			PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
		}



	}
	else if(m_mode == 2){
		// robot move rotation
		if(Linear.vx==0 && Linear.vy==0){
			if(Angular.vz > 0){
				Run_Dir = 'L';
				Run_State = 'U';
			}
			else if(Angular.vz < 0){
				Run_Dir = 'R';
				Run_State = 'U';
			}
			PWM_R(TURN(Angular.vz));
			PWM_L(TURN(Angular.vz));
		}
*/
/*
	// 좌회전
	if(m_mode == 1){
		Run_State = 'U';
		PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) + CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
		PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) - CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
	}
	// 우회전
	else if(m_mode == 2){
		Run_State = 'U';
		PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) - CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
		PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) + CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
	}
	else {
		// robot stop
		Run_State = 'B';
	}*/

	pwm_p = CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) + CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz);
	pwm_m = CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) - CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz);

	// 좌회전
	if(m_mode == 0){
		Run_State = 'B';
		PWM_R(0);
		PWM_L(0);
	}
	// 우회전
	else if(m_mode == 1){
		Run_State = 'U';
//		Run_Dir = 'F';

		if(pwm_m<0){
			Run_Dir = 'R';
		}
		else if(pwm_m>=0){
			Run_Dir = 'F';
		}
		PWM_R(fabs(pwm_m));
		PWM_L(pwm_p);
	}
	else if(m_mode == 2){
		Run_State = 'U';
//		Run_Dir = 'F';
		if(pwm_m<0){
			Run_Dir = 'L';
		}
		else if(pwm_m>=0){
			Run_Dir = 'F';
		}
		PWM_R(pwm_p);
		PWM_L(fabs(pwm_m));
	}
	else if(m_mode == 3){
		Run_State = 'U';
//		Run_Dir = 'O';
		if(pwm_m<0){
			Run_Dir = 'R';
		}
		else if(pwm_m>=0){
			Run_Dir = 'O';
		}
		PWM_R(pwm_p);
		PWM_L(fabs(pwm_m));
	}
	else if(m_mode == 4){
		Run_State = 'U';
//		Run_Dir = 'O';
		if(pwm_m<0){
			Run_Dir = 'L';
		}
		else if(pwm_m>=0){
			Run_Dir = 'O';
		}
		PWM_R(fabs(pwm_m));
		PWM_L(pwm_p);
	}
	else if(m_mode == 5){
		Run_State = 'U';
		Run_Dir = 'F';
		PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
		PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
	}
	else if(m_mode == 6){
		Run_State = 'U';
		Run_Dir = 'O';
		PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
		PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz));
	}








		/*
		angle = CAL_ANG(Linear.vx, Linear.vy, Linear.vz);
		TURN(angle);*/
//	}

	// Motor activate
//    RUN_DIR('F');
//    RUN_RB('U');
    RUN_RB(Run_State);
    MOTOR_SS(Motor_State);
    RUN_DIR(Run_Dir);

    rclc_executor_spin_some(&executor, timeout);


	  motor1.position_p = motor1.position;
	  motor2.position_p = motor2.position;


        osDelay(1-1);
      }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

	// 기본 ?��?��(default task) ?��?��?�� 구독 콜백 ?��?��
	void messageCallback(const void *msgin) {
		const std_msgs__msg__String_sub1 *msg = (const std_msgs__msg__String_sub1 *)msgin;
		printf("Received: %s\n", msg->dir_1);
	}

	void messageCallback_test1(const void *msgin) {
		const std_msgs__msg__Int32_test2 *msg = (const std_msgs__msg__Int32_test2 *)msgin;
		printf("Received: %d\n", msg->test_2);
	}

	void messageCallback_test2(const void *msgin) {
		const std_msgs__msg__Int32_test3 *msg = (const std_msgs__msg__Int32_test3 *)msgin;
		printf("Received: %d\n", msg->test_3);
	}

	void twist_subscription_callback(const void * msgin)
	{
	    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

	    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

/*		Run_State = 'U';
		Run_Dir = 'F';
		Motor_State = 'T';*/

//    	m_mode = 1;

/*	    if(m_mode == 0){
	    	m_mode = 1;
	    }
	    else{
	    	m_mode =0;
	    }*/

	    // 선속도와 각속도의 모든 값을 변수에 저장
/*	    float linear_velocity_x = msg->linear.x;
	    float linear_velocity_y = msg->linear.y;
	    float linear_velocity_z = msg->linear.z;

	    float angular_velocity_x = msg->angular.x;
	    float angular_velocity_y = msg->angular.y;
	    float angular_velocity_z = msg->angular.z;*/


/*		Linear.vx = submsg_2.linear.x;
		Linear.vy = submsg_2.linear.y;
		Linear.vz = submsg_2.linear.z;

		Angular.vx = submsg_2.angular.x;
		Angular.vy = submsg_2.angular.y;
		Angular.vz = submsg_2.angular.z;*/

	    // 선속도와 각속도의 모든 값을 변수에 저장
/*	    linear_velocity_x = msg->linear.x;
	    linear_velocity_y = msg->linear.y;
	    linear_velocity_z = msg->linear.z;

	    angular_velocity_x = msg->angular.x;
	    angular_velocity_y = msg->angular.y;
	    angular_velocity_z = msg->angular.z;*/


		Linear.vx = msg->linear.x;
		Linear.vy = msg->linear.y;
		Linear.vz = msg->linear.z;

		Angular.vx = msg->angular.x;
		Angular.vy = msg->angular.y;
		Angular.vz = msg->angular.z;

		MOTION_MODE(Linear.vx, Linear.vy, Linear.vz, Angular.vx, Angular.vy, Angular.vz);
/*		if(Linear.vy != 0)	m_mode = 2;
		else if(Linear.vy == 0){
			if(Linear.vx == 0)	m_mode = 0;
			else		m_mode = 1;
		}*/




	    // 저장된 값을 출력
/*	    printf("Received Twist - Linear Velocity (X: %f, Y: %f, Z: %f), Angular Velocity (X: %f, Y: %f, Z: %f)\n",
	           linear_velocity_x, linear_velocity_y, linear_velocity_z,
	           angular_velocity_x, angular_velocity_y, angular_velocity_z);*/

	    // MOTION_MODE 함수를 호출하여 m_mode 설정
//		m_mode = MOTION_MODE(linear_velocity_x, linear_velocity_y, linear_velocity_z);
//		MOTION_MODE(Linear.vx, Linear.vy, Linear.vz);
//		m_mode = MOTION_MODE(Linear.vx, Linear.vy, Linear.vz);
//		m_mode = 1;
	}

	// Motor function
	void RUN_INIT(void){
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, SET);	// Motor1 RUN/BRK - BRK
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, SET);	// Motor1 START/STOP - STOP
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, SET);	// Motor2 RUN/BRK - BRK
	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);	// Motor2 START/STOP - STOP
	}


	void RUN_DIR(char dir){

		if(dir == 'F'){			// Front
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, SET);		// Motor1 DIR - CW
		    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, RESET);	// Motor2 DIR - CCW
		}
		else if(dir == 'O'){	// Back
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, RESET);		// Motor1 DIR - CCW
		    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, SET);	// Motor2 DIR - CW
		}
		else if(dir == 'L'){	// Left
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, SET);		// Motor1 DIR - CW
		    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, SET);	// Motor2 DIR - CW
		}
		else if(dir == 'R'){	// Right
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, RESET);		// Motor1 DIR - CCW
		    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, RESET);	// Motor2 DIR - CCW
		}
	}

	void RUN_RB(char runstate){

		if(runstate == 'U'){
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, RESET);	// Motor1 RUN/BRK - RUN
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, RESET);	// Motor2 RUN/BRK - RUN
		}
		else if(runstate == 'B'){
		  	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, SET);	// Motor1 RUN/BRK - BRK
		  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, SET);	// Motor2 RUN/BRK - BRK
		}
	}

	void MOTOR_SS(char motorstate){

		if(motorstate == 'T'){
		    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, RESET);	// Motor1 START/STOP - START
		    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);	// Motor2 START/STOP - START
		}
		else if(motorstate == 'P'){
		  	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, SET);	// Motor1 START/STOP - STOP
		  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);	// Motor2 START/STOP - STOP
		}
	}

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
		if ( htim->Instance == TIM2){

			// Motor1
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				// - CW
				if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12) == 1){
					hall1 = hall1 + 1;
				}
				// - CCW
				else if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12) == 0){
					hall1 = hall1 - 1;
				}
			}
		}

		if ( htim->Instance == TIM5){

					// Motor2
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				// - CW
				if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2) == 1){
					hall2 = hall2 - 1;
				}
				// - CCW
				else if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2) == 0){
					hall2 = hall2 + 1;
				}
			}
		}






	}

/*	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if( htim->Instance == TIM3 ){
			if(turn_flag == 1){
				count_sec++;
			}
		}
*/

	void PWM_R(double duty){
		if(duty<=0){
			duty = 0;
		}
		else if(duty>=100){
			duty = 100;
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (int)duty-1);
	}
	void PWM_L(double duty){
		if(duty<=0){
			duty = 0;
		}
		else if(duty>=100){
			duty = 100;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int)duty-1);
	}

	// Calculate the final speed.
	double CAL_LinearSpeed(double x, double y, double z)
	{
		double distance = 0;
		double duty_percent = 0;

		if(x < 0){
			distance = -x;
		}
		else{
			distance = x;
		}

		duty_percent = distance * (20.0/(0.0965*M_PI));

		return duty_percent;
	}

	// Calculate the angle at which the robot should turn.

	double CAL_AngularSpeed(double x, double y, double z)
	{
		double wheel_pwm = 0;
		double angular = 0;

		if(z < 0){
					angular = -z;
				}
		else{
			angular = z;
		}

		wheel_pwm = (100*0.38*angular)/(5*M_PI*0.0965*2);

		return wheel_pwm;
	}

	void MOTION_MODE(double lx, double ly, double lz, double ax, double ay, double az)
	{
		/*
	    if(lx == 0 && az == 0) {
	        m_mode = 0; // 정지
	    }
	    else {
	    	if(az >= 0){
	    		if(lx > 0){
	    			m_mode = 2; // 2사분면
	    		}
	    		else if(lx < 0){
	    			m_mode = 4; // 4사분면
	    		}
	    	}
	    	else if(az < 0){
	    		if(lx >= 0){
	    			m_mode = 1; // 1사분면
	    		}
	    		else if(lx < 0){
	    			m_mode = 3; // 3사분면
	    		}
	    	}
	    }
	    */

	    if(lx == 0 && az == 0) {
	        m_mode = 0; // 정지
	    }
	    else{
	    	if(az == 0){
	    		if(lx>0){
	    			m_mode = 5; // 직진
	    		}
	    		else if(lx<0){
	    			m_mode = 6; // 후진
	    		}
	    	}
	    	else if(az > 0){
	    		if(lx >= 0){
	    			m_mode = 2; // 2사분면
	    		}
	    		else if(lx < 0){
	    			m_mode = 4; // 4사분면
	    		}
	    	}
	    	else if(az < 0){
	    		if(lx >= 0){
	    			m_mode = 1; // 1사분면
	    		}
	    		else if(lx < 0){
	    			m_mode = 3; // 3사분면
	    		}
	    	}
	    }
	}

	int16_t hex_to_signed_int(const char *hex_str){
		int16_t value = (int16_t)strtol(hex_str, NULL, 16);
		if(value > 0x7FFF){
			value -= 0x10000;
		}
		return value;
	}

	/*
	void MOTION_MODE(double lx, double ly, double lz, double ax, double ay, double az)
	{
	    if(lx == 0 && ly == 0) {
	        m_mode = 0; // 정지
	    }
	    else {
	    	if(az < 0){
		        m_mode = 2;
	    	}
	    	else{
	    		m_mode = 1;
	    	}
	    }
	}
*/
/*
	void MOTION_MODE(double lx, double ly, double lz, double ax, double ay, double az)
	{
		if(az != 0)	m_mode = 2;
		else if(ly == 0){
			if(lx == 0)	m_mode = 0;
			else		m_mode = 1;
		}
	}
	*/
/*
	void MOTION_MODE(double lx, double ly, double lz, double ax, double ay, double az)
		{
			if(y != 0)	m_mode = 2;
			else if(y == 0){
				if(x == 0)	m_mode = 0;
				else		m_mode = 1;
			}
		}*/
/*
	int MOTION_MODE(double x, double y, double z)
	{
		if(y != 0)	return 2;
		else if(y == 0){
			if(x == 0)	return 0;
			else		return 1;
		}
	}
*/
	/*
	int MOTION_MODE(double x, double y, double z)
	{
	    if(y != 0)	return 2;
	    if(x == 0)	return 0;
	    return 1;
	}
*/








/*	double MOTOR_CAL_LINE_PWM(double x, double y, double z){
	}
*/




































/* USER CODE END Application */

