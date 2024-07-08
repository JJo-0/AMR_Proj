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

#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

#define _USE_MATH_DEFINES

char Run_State;
char Motor_State;
char Run_Dir;


// 선속도와 각속도의 모든 값을 변수에 저장
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

int hall1 = 0, hall2 = 0;

typedef struct{
	double vx, vy, vz;
} Twist_value;
Twist_value Linear;
Twist_value Angular;

int m_mode = 0; // 0: stop, 1: straight & turn left, 2: straight & turn right

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

void RUN_DIR(char);

void RUN_RB(char);
void MOTOR_SS(char);

double MOTOR_CAL_LINE_PWM(double,double,double);

void PWM_L(double);
void PWM_R(double);

double CAL_LinearSpeed(double, double, double);
double CAL_AngularSpeed(double x, double y, double z);

double TURN(double ang);
void MOTION_MODE(double lx, double ly, double lz, double ax, double ay, double az);

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

  // micro-ROS app
  // create publisher
  rcl_publisher_t publisher_1;
  std_msgs__msg__Int32_pub1 pubmsg_1;

  rcl_publisher_t publisher_2;
  std_msgs__msg__Int32_pub2 pubmsg_2;
  // create subscriber
  rcl_subscription_t subscriber_2;
  geometry_msgs__msg__Twist submsg_2;

  // node
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);



  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // create publisher_2
  rclc_publisher_init_default(
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

  	  //l_enc

  // create subscriber_test1
  rclc_subscription_init_default(
      &subscriber_2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "diffbot_base_controller/cmd_vel_unstamped");

  // add subscriber to executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(&executor, &subscriber_2, &submsg_2, &twist_subscription_callback, ON_NEW_DATA);

  pubmsg_1.hall_1 = 0;
  pubmsg_2.hall_2 = 0;

  for(;;)
  {
    // publish message
	 rcl_ret_t ret_3 = rcl_publish(&publisher_2, &pubmsg_2, NULL);
	 rcl_ret_t ret_2 = rcl_publish(&publisher_1, &pubmsg_1, NULL);
    if (ret_2 != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }
    if (ret_3 != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }

    // subscribe message

    // hall pulse count
	pubmsg_1.hall_1 = hall1;
	pubmsg_2.hall_2 = hall2;

    // reset the hall sensor data
	hall1 = 0;
	hall2 = 0;

	// 좌회전
	if(m_mode == 1){
		PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) + CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
		PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) - CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
	}
	// 우회전
	else if(m_mode == 2){
		PWM_R(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) - CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
		PWM_L(CAL_LinearSpeed(Linear.vx, Linear.vy, Linear.vz) + CAL_AngularSpeed(Angular.vx, Angular.vy, Angular.vz));
	}
	else {
		// robot stop
		Run_State = 'B';
	}
	// Motor activate
    RUN_DIR(Run_Dir);
    RUN_RB(Run_State);
    MOTOR_SS(Motor_State);

    rclc_executor_spin_some(&executor, timeout);

        osDelay(10);
      }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

	// 기본 ?��?��(default task) ?��?��?�� 구독 콜백 ?��?��
	void twist_subscription_callback(const void * msgin)
	{
	    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

	    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		Linear.vx = msg->linear.x;
		Linear.vy = msg->linear.y;
		Linear.vz = msg->linear.z;

		Angular.vx = msg->angular.x;
		Angular.vy = msg->angular.y;
		Angular.vz = msg->angular.z;

		MOTION_MODE(Linear.vx, Linear.vy, Linear.vz, Angular.vx, Angular.vy, Angular.vz);
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


	void PWM_R(double duty){
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (int)duty-1);
	}
	void PWM_L(double duty){
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

	double CAL_AngularSpeed(double x, double y, double z)
	{
		double wheel_pwm = 0;

		wheel_pwm = (100*0.38*z)/(5*M_PI*0.0965*2);

		return wheel_pwm;
	}

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

/* USER CODE END Application */

