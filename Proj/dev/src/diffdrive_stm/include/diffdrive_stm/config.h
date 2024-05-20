#ifndef DIFFDRIVE_STM_CONFIG_H
#define DIFFDRIVE_STM_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30;
  int timeout = 1000;
  int enc_counts_per_rev = 1920;
};


#endif // DIFFDRIVE_STM_CONFIG_H