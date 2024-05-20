#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "diffdrive_stm/diffdrive_stm.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diffdrive_robot");
  ros::NodeHandle n("~");

  DiffDriveStm::Config robot_cfg;

  // Attempt to retrieve parameters. If they don't exist, the default values from the struct will be used
  n.getParam("left_wheel_name", robot_cfg.left_wheel_name);
  n.getParam("right_wheel_name", robot_cfg.right_wheel_name);
  n.getParam("enc_counts_per_rev", robot_cfg.enc_counts_per_rev);
  n.getParam("robot_loop_rate", robot_cfg.loop_rate);
  

  DiffDriveStm robot(robot_cfg);
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prevTime = ros::Time::now();

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();

    loop_rate.sleep();
  }
}
