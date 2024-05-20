#ifndef DIFFDRIVE_STM_WHEEL_H
#define DIFFDRIVE_STM_WHEEL_H

#include <string>



class Wheel
{
    public:

    std::string name = "";
    int enc = 0;    //엔코더 값
    double cmd = 0;   //
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev);
    
    void setup(const std::string &wheel_name, int counts_per_rev);

    double calcEncAngle();



};


#endif // DIFFDRIVE_STM_WHEEL_H