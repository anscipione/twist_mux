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
 */

#include <twist_mux/twist_mux_diagnostics.h>
#include <twist_mux/twist_mux_diagnostics_status.h>

#include <diagnostic_updater/diagnostic_updater.hpp>

namespace twist_mux
{

TwistMuxDiagnostics::TwistMuxDiagnostics(rclcpp::Node *n)  //const rclcpp::NodeOptions & options)
: n_(n) //Node("TwistMuxDiagnostics", options)
, diagnostic_(n_)
, status_(n_->now())
{
  diagnostic_.add("Twist mux status", this, &TwistMuxDiagnostics::diagnostics);
  diagnostic_.setHardwareID("none");
}

TwistMuxDiagnostics::~TwistMuxDiagnostics()
{}

void TwistMuxDiagnostics::update()
{
  diagnostic_.force_update();
}

void TwistMuxDiagnostics::updateStatus(const status_type::ConstPtr& status)
{

  auto& clk = *n_->get_clock();
  RCLCPP_DEBUG_THROTTLE(n_->get_logger(),clk,1.0, "Updating status.");

  status_.velocity_hs = status->velocity_hs;
  status_.lock_hs     = status->lock_hs;
  status_.priority    = status->priority;

  status_.main_loop_time   = status->main_loop_time;
  status_.reading_age      = status->reading_age;
  status_.last_loop_update = status->last_loop_update;

 
  update();
  
}

void TwistMuxDiagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  /// Check if the loop period is quick enough
  if (status_.main_loop_time > MAIN_LOOP_TIME_MIN)
    stat.summary(ERROR, "loop time too long");
  else if (status_.reading_age > READING_AGE_MIN)
    stat.summary(ERROR, "data received is too old");
  else
    stat.summary(OK, "ok");

  for (const auto& velocity_h : *status_.velocity_hs)
  {
    stat.addf("velocity " + velocity_h.getName(),
              " %s (listening to %s @ %fs with priority #%d)",
              (velocity_h.isMasked(status_.priority) ? "masked" : "unmasked"),
              velocity_h.getTopic().c_str(),
              velocity_h.getTimeout(),
              static_cast<int>(velocity_h.getPriority()));
  }

  for (const auto& lock_h : *status_.lock_hs)
  {
    stat.addf("lock " + lock_h.getName(),
              " %s (listening to %s @ %fs with priority #%d)",
              (lock_h.isLocked() ? "locked" : "free"),
              lock_h.getTopic().c_str(),
              lock_h.getTimeout(),
              static_cast<int>(lock_h.getPriority()));
  }

  stat.add("current priority", static_cast<int>(status_.priority));

  stat.add("loop time in [sec]", status_.main_loop_time);
  stat.add("data age in [sec]", status_.reading_age);

  auto& clk = *n_->get_clock();
  RCLCPP_DEBUG_THROTTLE(n_->get_logger(),clk,1.0, "Publishing diagnostics.");
  
 //RCLCPP_INFO(n_->get_logger(), "Publishing diagnostics.");
}

} // namespace twist_mux


//#include "rclcpp_components/register_node_macro.hpp"
//
//// Register the component with class_loader.
//// This acts as a sort of entry point, allowing the component to be discoverable when its library
//// is being loaded into a running process.
//RCLCPP_COMPONENTS_REGISTER_NODE(twist_mux::TwistMuxDiagnostics)

