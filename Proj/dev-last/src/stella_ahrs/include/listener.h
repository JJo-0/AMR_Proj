#include <iostream>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <pthread.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "MwSerial.hpp"

#define DEG2RAD( a ) ( (a) * (M_PI/180.0f) )
#define COS(a) cos(DEG2RAD(a))
#define SIN(a) sin(DEG2RAD(a))

#define DeviceID 0x01
#define STX 0x02
#define ETX 0x03
#define Command 0xF0

#define ACC 0x33
#define GYO 0x34
#define DEG 0x35

using namespace std;

char data[8];
long id = 0;
int length = 0;
bool run = true;

auto imu = std::make_shared<sensor_msgs::msg::Imu>();
