#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace mc_rtc_ros_control
{

struct ROSControlInterfaceImpl;

struct ROSControlInterface
{
  ROSControlInterface(const std::string & subscribe_to,
                      const std::string & publish_to,
                      const std::vector<std::string> & ref_joint_order);

  ~ROSControlInterface();

  using state_callback_t =
      std::function<void(const std::vector<double> &, const std::vector<double> &, const std::vector<double> &)>;

  /** Set a callback to be called whenever a new state is received */
  void onStateCallback(state_callback_t callback);

  /** Write on the command topic */
  void sendCommand(const std::vector<double> & command);

private:
  std::unique_ptr<ROSControlInterfaceImpl> impl_;
};

} // namespace mc_rtc_ros_control
