#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mc_control/mc_global_controller.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#ifdef SPDLOG_FMT_EXTERNAL
#  include <fmt/ranges.h>
#else
#  include <spdlog/fmt/bundled/ranges.h>
#endif

#include <mc_rtc_ros_control/ROSControlInterface.h>

enum class ControlOutput
{
  Position,
  Velocity,
  Torque
};

template<ControlOutput OutT>
struct ROSControlInterface
{
  void onRobotState(mc_rtc_ros_control::ROSControlInterface & interface,
                    const std::vector<double> & encoders,
                    const std::vector<double> & velocity,
                    const std::vector<double> & efforts)
  {
    controller_.setEncoderValues(encoders);
    controller_.setEncoderVelocities(velocity);
    controller_.setJointTorques(efforts);
    if(!init_done_)
    {
      controller_.init(controller_.robot().encoderValues());
      controller_.running = true;
      init_done_ = true;
    }
    else
    {
      if(controller_.run())
      {
        const auto & robot = controller_.robot();
        const auto & mbc = robot.mbc();
        const auto & command = [&]() -> const std::vector<std::vector<double>> & {
          if constexpr(OutT == ControlOutput::Position)
          {
            command_ = encoders;
            return mbc.q;
          }
          else if constexpr(OutT == ControlOutput::Velocity)
          {
            command_ = velocity;
            return mbc.alpha;
          }
          else if constexpr(OutT == ControlOutput::Torque)
          {
            command_ = efforts;
            return mbc.jointTorque;
          }
          else
          {
            static_assert(static_cast<int>(OutT) > 255, "Not ok");
          }
        }();
        for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
        {
          auto jIndex = robot.jointIndexInMBC(i);
          if(jIndex != -1)
          {
            command_[i] = command[jIndex][0];
          }
        }
        interface.sendCommand(command_);
      }
    }
  }

  inline const std::vector<std::string> & rjo() const noexcept
  {
    return controller_.robot().refJointOrder();
  }

private:
  mc_control::MCGlobalController controller_;
  std::vector<double> command_;
  bool init_done_ = false;
};

template<ControlOutput output>
void run()
{
  ROSControlInterface<output> interface;
  mc_rtc_ros_control::ROSControlInterface ros_iface("subscribe_to", "publish_to", interface.rjo());
  ros_iface.onStateCallback([&](const std::vector<double> & e, const std::vector<double> & v,
                                const std::vector<double> & t) { interface.onRobotState(ros_iface, e, v, t); });
  rclcpp::spin(ros_iface.nh);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error(
        "mc_rtc_ros_control was compiled with {} but mc_rtc is at version {}, you might face subtle issues "
        "or unexpected crashes, please recompile mc_rtc_ros_control",
        mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  rclcpp::NodeOptions options;
  rclcpp::Node node("mc_rtc_ros_control", options);
  bool output_velocity = node.declare_parameter<bool>("output_velocity", false);
  bool output_torque = node.declare_parameter<bool>("output_torque", false);
  if(output_velocity && output_torque)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Only one of output_velocity or output_torque can be true");
  }
  if(!output_velocity && !output_torque)
  {
    run<ControlOutput::Position>();
  }
  else if(output_velocity)
  {
    run<ControlOutput::Velocity>();
  }
  else
  {
    run<ControlOutput::Torque>();
  }
  return 0;
}
