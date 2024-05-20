#include "diffdrive_stm/my_node.h"


Publisher::Publisher() : Node("vel_publisher"){
    RCLCPP_INFO(get_logger(), "Cmd_vel Pub Node Created");
    l_wheel_vel_pub = create_publisher<std_msgs::msg::Int32>("/l_wheel_vel", 10);
    r_wheel_vel_pub = create_publisher<std_msgs::msg::Int32>("/r_wheel_vel", 10);
}
void Publisher::setMotorValues(int val_1, int val_2) {
    RCLCPP_INFO(get_logger(), "setmotor");
    l_wheel_vel_pub->publish(val_1);
    r_wheel_vel_pub->publish(val_2);
}


//-----------------------------------------------------------------------------------------------------------//

Subscriber::Subscriber() : Node("enc_subscriber")
{
    RCLCPP_INFO(get_logger(), "enc Sub Node Created");
  subscription_1 = this->create_subscription<std_msgs::msg::Int32>(
  "l_enc", 10, std::bind(&Subscriber::topic_callback_l, this, std::placeholders::_1));

  subscription_2 = this->create_subscription<std_msgs::msg::Int32>(
  "r_enc", 10, std::bind(&Subscriber::topic_callback_r, this, std::placeholders::_1));
}
void Subscriber::topic_callback_l(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "callback_l");
  l_wheel_enc = msg->data;
}
void Subscriber::topic_callback_r(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "callback_R");
  r_wheel_enc = msg->data;
}
