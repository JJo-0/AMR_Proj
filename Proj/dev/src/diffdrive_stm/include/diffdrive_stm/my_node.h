#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class Subscriber : public rclcpp::Node {
  public:
    int l_wheel_enc;
    int r_wheel_enc;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_2;

    Subscriber();

    void topic_callback_l(const std_msgs::msg::Int32::SharedPtr msg);
    void topic_callback_r(const std_msgs::msg::Int32::SharedPtr msg);
};
class Publisher : public rclcpp::Node {
public:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr l_wheel_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr r_wheel_vel_pub;

    Publisher();

    void setMotorValues(int val_1, int val_2);
};