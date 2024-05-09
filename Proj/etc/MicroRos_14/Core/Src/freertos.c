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
#include <std_msgs/msg/string.h>

#include "usart.h"
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

char Run_State = 'U';
char Motor_State = 'P';
char Run_Dir = 'F';



int m1_hall=0, m2_hall=0;

int m1_rev=0, m2_rev=0;
int m1_deg=0, m2_deg=0;
int m1_deg_10=0, m2_deg_10=0;
int m1_deg_10_p=0, m2_deg_10_p=0;
int m1_deg_1_10=0, m2_deg_1_10=0;
//float m1_deg=0, m2_deg=0;
//float m1_deg_10=0, m2_deg_10=0;
//float m1_deg_1_10=0, m2_deg_1_10=0;
int m1_rpm=0, m2_rpm=0;
int m1_rpm_p=0, m2_rpm_p=0;
int m1_rpm_p_10=0, m2_rpm_p_10=0;

//std_msgs__msg__Int32 msg;
int hall1 = 0, hall2 = 0;


int count_sec = 0;

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

//my motor functions
void RUN_DIR(char);
void RUN_RB(char);
void MOTOR_SS(char);





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

  rcl_subscription_t subscriber_1;
  std_msgs__msg__String_sub1 submsg_1;

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
  	 // l_wheel_vel
  // create publisher_2
  rclc_publisher_init_default(
    &publisher_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "l_enc");
  	  // r_wheel_vel
  // create publisher_3
  rclc_publisher_init_default(
    &publisher_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "r_enc");
  	  //l_enc
  // create publisher_test
  rclc_publisher_init_default(
    &publisher_test,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cubemx_publisher_test");

  // create subscriber_test1
  rclc_subscription_init_default(
      &subscriber_test1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//	    "cubemx_publisher_test");
        "l_wheel_vel");

  // create subscriber_test2
  rclc_subscription_init_default(
      &subscriber_test2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//	    "cubemx_publisher_test");
        "r_wheel_vel");

  // create subscriber
  rclc_subscription_init_default(
      &subscriber_1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "r_wheel_vel_String");




  // add subscriber to executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber_1, &submsg_1, &messageCallback, ON_NEW_DATA);

  rclc_executor_add_subscription(&executor, &subscriber_test1, &submsg_test1, &messageCallback_test1, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_test2, &submsg_test2, &messageCallback_test2, ON_NEW_DATA);

  msg.data = 0;
  pubmsg_1.hall_1 = 0;

  for(;;)
  {
    // publish message
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    rcl_ret_t ret_2 = rcl_publish(&publisher_1, &pubmsg_1, NULL);
    rcl_ret_t ret_3 = rcl_publish(&publisher_2, &pubmsg_2, NULL);
//    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    rcl_ret_t ret_test = rcl_publish(&publisher_test, &msg_test, NULL);
    if (ret != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__); 
    }
//    msg.data++;

    // Motor Controll
/*    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, RESET);	// Motor1 DIR - CCW
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, RESET);	// Motor1 RUN/BRK - RUN
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, RESET);	// Motor1 START/STOP - START

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, SET);	// Motor2 DIR - CW
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, RESET);	// Motor2 RUN/BRK - RUN
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);	// Motor2 START/STOP - START
*/


//    msg.data = abc;
    /*
	msg.hall_1 = hall1;
	msg.hall_2 = hall2;
*/

	pubmsg_1.hall_1 = hall1;
	pubmsg_2.hall_2 = hall2;




/*
    if(msg.data < 1000){
    	Run_Dir = 'F';
    	Run_State = 'U';
    }
    else if((msg.data >= 1000)&&(msg.data <= 2000)){
    	Run_State = 'B';
    }
    else if((msg.data >= 2000)&&(msg.data <= 3000)){
    	Run_Dir = 'O';
    	Run_State = 'U';
    }
*/
/*
    if(msg.hall_1 < 1000){
    	Run_Dir = 'F';
    	Run_State = 'U';
    }
    else if((msg.hall_1 >= 1000)&&(msg.hall_1 <= 2000)){
    	Run_State = 'B';
    }
    else if((msg.hall_1 >= 2000)&&(msg.hall_1 <= 3000)){
    	Run_Dir = 'O';
    	Run_State = 'U';
    }
*/
/*
    if(msg.hall_2 < 1000){
    	Run_Dir = 'F';
    	Run_State = 'U';
    }
    else if((msg.hall_2 >= 1000)&&(msg.hall_2 <= 2000)){
    	Run_State = 'B';
    }
    else if((msg.hall_2 >= 2000)&&(msg.hall_2 <= 3000)){
    	Run_Dir = 'O';
    	Run_State = 'U';
    }
*/
/*
    if(msg_2.hall_1 < 1000){
    	Run_Dir = 'F';
    	Run_State = 'U';
    }
    else if((msg_2.hall_1 >= 1000)&&(msg_2.hall_1 <= 2000)){
    	Run_State = 'B';
    }
    else if((msg_2.hall_1 >= 2000)&&(msg_2.hall_1 <= 3000)){
    	Run_Dir = 'O';
    	Run_State = 'U';
    }
*/
    RUN_DIR(Run_Dir);
    RUN_RB(Run_State);

        osDelay(10);
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
		printf("Recieved: %d\n", msg->test_2);

	}

	void messageCallback_test2(const void *msgin) {
		const std_msgs__msg__Int32_test3 *msg = (const std_msgs__msg__Int32_test3 *)msgin;
		printf("Recieved: %d\n", msg->test_3);

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
		    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, SET);	// Motor2 DIR - CCW
		}
		else if(dir == 'R'){	// Right
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, RESET);		// Motor1 DIR - CW
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
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				hall1 = hall1 + 1;

				/*
				m1_deg_10 = m1_hall * 15;
				m1_deg = m1_deg_10 / 10;
				m1_deg_1_10 = m1_deg_10 % 10;

				m1_rpm = (m1_deg_10 - m1_deg_10_p) * 60 / (1 * 10);

				m1_deg_10_p = m1_deg_10;
	*/

/*
				if(m1_hall==240)
				{
					m1_hall = 0;



	//				m1_deg++;

					m1_rev++;
				}
*/

	/*		  	printf("%d \n", m1_deg);
				printf("%d \n", m1_rev);
				printf("\n");*/
			}

			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				hall2 = hall2 + 1;
			}



		}







	}







/* USER CODE END Application */

