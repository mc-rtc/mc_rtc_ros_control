#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <mc_control/mc_global_controller.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#ifdef SPDLOG_FMT_EXTERNAL
#  include <fmt/ranges.h>
#else
#  include <spdlog/fmt/bundled/ranges.h>
#endif

struct ROSControlInterface
{
  ROSControlInterface()
  {
    msg_.layout.dim.resize(1);
    msg_.layout.dim[0].label = "control";
    msg_.layout.dim[0].size = rjo().size();
    msg_.layout.dim[0].stride = msg_.layout.dim[0].size;
    msg_.data.resize(msg_.layout.dim[0].size);
    msg_.layout.data_offset = 0;
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("publish_to", 1);
    sub_ = nh_.subscribe("subscribe_to", 1, &ROSControlInterface::init, this);
  }

  void init(const sensor_msgs::JointState & msg)
  {
    if(msg.name != rjo())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Look like mc_rtc_ros_control is subscribed to a different "
                                                       "robot than what it expects (got: {}, expected: {})",
                                                       msg.name, rjo());
    }
    updateSensors(msg);
    controller_.init(msg.position);
    controller_.running = true;
    sub_ = nh_.subscribe("subscribe_to", 1, &ROSControlInterface::run, this);
  }

  void run(const sensor_msgs::JointState & msg)
  {
    updateSensors(msg);
    if(controller_.run())
    {
      const auto & q = controller_.robot().mbc().q;
      const auto & rjo = this->rjo();
      for(size_t i = 0; i < rjo.size(); ++i)
      {
        const auto & j = rjo[i];
        auto jIndex = controller_.robot().jointIndexInMBC(i);
        if(jIndex != -1)
        {
          msg_.data[i] = q[i][0];
        }
        else
        {
          msg_.data[i] = msg.position[i];
        }
      }
      pub_.publish(msg_);
    }
  }

  void updateSensors(const sensor_msgs::JointState & msg)
  {
    controller_.setEncoderValues(msg.position);
    controller_.setEncoderVelocities(msg.velocity);
    controller_.setJointTorques(msg.effort);
  }

  inline const std::vector<std::string> & rjo() const noexcept
  {
    return controller_.robot().module().ref_joint_order();
  }

private:
  mc_control::MCGlobalController controller_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std_msgs::Float64MultiArray msg_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "mc_rtc_ros_control");

  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error(
        "mc_rtc_ros_control was compiled with {} but mc_rtc is at version {}, you might face subtle issues "
        "or unexpected crashes, please recompile mc_rtc_ros_control",
        mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  ROSControlInterface interface;
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
