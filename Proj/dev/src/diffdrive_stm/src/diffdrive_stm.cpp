#include "diffdrive_stm/diffdrive_stm.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

//--_____------------------------------------------------------------------------------------------------//


//--------------------------------------------------------------------------------------------------------------

DiffDriveStm::DiffDriveStm()
    : logger_(rclcpp::get_logger("DiffDriveStm"))
{
}


return_type DiffDriveStm::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveStm::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveStm::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


return_type DiffDriveStm::start()
{
  // 컨트롤러 시작 로그 출력
  RCLCPP_INFO(logger_, "Starting Controller...");


  // 컨트롤러의 상태를 STARTED로 변경
  status_ = hardware_interface::status::STARTED;

  // 모든 초기화가 성공적으로 완료되면 OK 반환
  return return_type::OK;
}

return_type DiffDriveStm::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}


// DiffDriveStm 클래스의 read 메서드 구현
hardware_interface::return_type DiffDriveStm::read()
{
  // 현재 시간을 new_time 변수에 저장하여 시간 차이(델타 시간)를 계산하기 위한 준비
  auto new_time = std::chrono::system_clock::now();
  // 이전 시간(time_)과 현재 시간(new_time)의 차이를 계산
  std::chrono::duration<double> diff = new_time - time_;
  // 계산된 시간 차이를 초 단위로 변환하여 deltaSeconds에 저장
  double deltaSeconds = diff.count();
  // 현재 시간을 time_ 변수에 업데이트하여 다음 계산을 위해 저장
  time_ = new_time;

  // 왼쪽 바퀴의 이전 위치를 pos_prev에 저장
  double pos_prev = l_wheel_.pos;
  // 왼쪽 바퀴의 새 위치를 계산하여 pos에 저장
  l_wheel_.pos = l_wheel_.calcEncAngle();
  // 왼쪽 바퀴의 속도를 계산 (새 위치 - 이전 위치) / 시간 차이
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  // 오른쪽 바퀴에 대해서도 동일한 과정 수행
  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;


  // 모든 처리가 성공적으로 완료되면, OK를 반환
  return return_type::OK;
}


// DiffDriveStm 클래스의 write 메서드 구현
hardware_interface::return_type DiffDriveStm::write()
{

  // 아두이노에 모터 값을 설정
  // l_wheel_.cmd는 왼쪽 바퀴에 대한 명령된 속도(라디안/초)
  // r_wheel_.cmd는 오른쪽 바퀴에 대한 명령된 속도(라디안/초)
  // l_wheel_.rads_per_count 및 r_wheel_.rads_per_count는 각 바퀴의 인코더 해상도(라디안/카운트)
  // cfg_.loop_rate는 제어 루프의 실행 빈도(Hz)
  // 따라서, cmd / rads_per_count / loop_rate 계산을 통해, 명령된 속도를 인코더 카운트/루프 실행으로 변환
  // 이 값은 아두이노에 전달되어 모터 드라이버에 의해 모터의 속도를 제어하는 데 사용됨
  // 아마 cmd는 각속도
  pub_cmd_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate,
                          r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

  // 모든 처리가 성공적으로 완료되면, OK를 반환
  return return_type::OK;
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveStm,
  hardware_interface::SystemInterface
)