/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 */

#ifndef TWIST_MUX_H
#define TWIST_MUX_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <list>

namespace twist_mux
{

// Forwarding declarations:
class TwistMuxDiagnostics;
struct TwistMuxDiagnosticsStatus;
class VelocityTopicHandle;
class LockTopicHandle;

/**
 * @brief The TwistMux class implements a top-level twist multiplexer module
 * that priorize different velocity command topic inputs according to locks.
 */
class TwistMux : public rclcpp::Node
{
public:

  // @todo use this type alias when the compiler supports this C++11 feat.
  //template<typename T>
  //using handle_container = std::list<T>;
  // @todo alternatively we do the following:
  typedef std::list<VelocityTopicHandle> velocity_topic_container;
  typedef std::list<LockTopicHandle>     lock_topic_container;

  TwistMux(const std::string name = "twist_mux_node");
  ~TwistMux();

  bool hasPriority(const VelocityTopicHandle& twist);

  void publishTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);

  void updateDiagnostics();

protected:

  typedef TwistMuxDiagnostics       diagnostics_type;
  typedef TwistMuxDiagnosticsStatus status_type;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  static constexpr int DIAGNOSTICS_PERIOD = 1;

  /**
   * @brief velocity_hs_ Velocity topics' handles.
   * Note that if we use a vector, as a consequence of the re-allocation and
   * the fact that we have a subscriber inside with a pointer to 'this', we
   * must reserve the number of handles initially.
   */
  // @todo use handle_container (see above)
  //handle_container<VelocityTopicHandle> velocity_hs_;
  //handle_container<LockTopicHandle> lock_hs_;
  std::shared_ptr<velocity_topic_container> velocity_hs_;
  std::shared_ptr<lock_topic_container>     lock_hs_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  cmd_pub_;

  geometry_msgs::msg::Twist last_cmd_;

  template<typename T>
  void getTopicHandles( const std::string& param_name, std::list<T>& topic_hs);

  int getLockPriority();

  std::shared_ptr<diagnostics_type> diagnostics_;
  std::shared_ptr<status_type>      status_;
  rclcpp::Duration period_;
  //rclcpp::TimerBase::SharedPtr update_timer_;
};

} // namespace twist_mux

#endif // TWIST_MUX_H
