#include <mc_rtc_ros_control/ROSControlInterface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace mc_rtc_ros_control
{

struct ROSControlInterfaceImpl
{
  ROSControlInterfaceImpl(const std::string & subscribe_to,
                          const std::string & publish_to,
                          const std::vector<std::string> & rjo)
  : rjo_(rjo)
  {
    msg_.layout.dim.resize(1);
    msg_.layout.dim[0].label = "control";
    msg_.layout.dim[0].size = rjo.size();
    msg_.layout.dim[0].stride = msg_.layout.dim[0].size;
    msg_.data.resize(msg_.layout.dim[0].size);
    msg_.layout.data_offset = 0;
    pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>(publish_to, 1);
    sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
        subscribe_to, 1, std::bind(&ROSControlInterfaceImpl::joint_callback, this, std::placeholders::_1));
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if(!init_done_)
    {
      init(msg);
    }
    else
    {
      run(msg);
    }
  }

  void init(const sensor_msgs::msg::JointState::SharedPtr & msg)
  {
    const auto & rjo = rjo_;
    if(msg->name.size() > rjo.size())
    {
      RCLCPP_FATAL(nh_->get_logger(),
                   "Joint state passed in to mc_rtc_ros_control has more joints than the robot reference joint order, "
                   "something is wrong");
    }
    msg_.layout.dim[0].size = msg->name.size();
    msg_.data.resize(msg_.layout.dim[0].size);
    ros_to_rjo_.resize(msg->name.size());
    rjo_to_cmd_.resize(rjo.size(), std::numeric_limits<size_t>::max());
    encoders_.resize(rjo.size(), 0.0);
    velocity_.resize(rjo.size(), 0.0);
    efforts_.resize(rjo.size(), 0.0);
    for(size_t i = 0; i < msg->name.size(); ++i)
    {
      const auto & n = msg->name[i];
      auto it = std::find(rjo.begin(), rjo.end(), n);
      if(it == rjo.end())
      {
        RCLCPP_FATAL(nh_->get_logger(), "Joint state passed in to mc_rtc_ros_control for %s which does not exist in the robot reference joint order, something is wrong",
                     n.c_str());
      }
      ros_to_rjo_[i] = std::distance(rjo.begin(), it);
    }
    size_t cmd_index = 0;
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      auto it = std::find(msg->name.begin(), msg->name.end(), rjo[i]);
      if(it != msg->name.end())
      {
        rjo_to_cmd_[i] = cmd_index;
        cmd_index++;
      }
    }
    init_done_ = true;
  }

  void run(const sensor_msgs::msg::JointState::SharedPtr & msg)
  {
    for(size_t i = 0; i < ros_to_rjo_.size(); ++i)
    {
      auto rjoIdx = ros_to_rjo_[i];
      encoders_[rjoIdx] = msg->position[i];
      velocity_[rjoIdx] = msg->velocity[i];
      efforts_[rjoIdx] = msg->effort[i];
    }
    if(callback_)
    {
      callback_(encoders_, velocity_, efforts_);
    }
  }

  void sendCommand(const std::vector<double> & command)
  {
    for(size_t i = 0; i < command.size(); ++i)
    {
      if(rjo_to_cmd_[i] < msg_.data.size())
      {
        msg_.data[rjo_to_cmd_[i]] = command[i];
      }
    }
    pub_->publish(msg_);
  }

  std::vector<std::string> rjo_;
  bool init_done_ = false;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  std_msgs::msg::Float64MultiArray msg_;
  std::vector<size_t> ros_to_rjo_;
  std::vector<size_t> rjo_to_cmd_;
  std::vector<double> encoders_;
  std::vector<double> velocity_;
  std::vector<double> efforts_;

  ROSControlInterface::state_callback_t callback_;
};

ROSControlInterface::ROSControlInterface(const std::string & subscribe_to,
                                         const std::string & publish_to,
                                         const std::vector<std::string> & ref_joint_order)
: impl_(new ROSControlInterfaceImpl(subscribe_to, publish_to, ref_joint_order))
{
  nh = impl_->nh_;
}

ROSControlInterface::~ROSControlInterface() = default;

void ROSControlInterface::onStateCallback(state_callback_t callback)
{
  impl_->callback_ = callback;
}

void ROSControlInterface::sendCommand(const std::vector<double> & command)
{
  if(!impl_->init_done_)
  {
    RCLCPP_FATAL(impl_->nh_->get_logger(), "ROSControlInterface::sendCommand called before initialization is done");
  }
  impl_->sendCommand(command);
}

} // namespace mc_rtc_ros_control
