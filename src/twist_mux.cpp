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

#include <twist_mux/twist_mux.h>
#include <twist_mux/topic_handle.h>
#include <twist_mux/twist_mux_diagnostics.h>
#include <twist_mux/twist_mux_diagnostics_status.h>
#include <twist_mux/utils.h>
#include "yaml-cpp/yaml.h"
//#include <twist_mux/xmlrpc_helpers.h>
#include <twist_mux/twistmux_utils.h>
#include <geometry_msgs/msg/twist_stamped.hpp>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_twist Old velocity
 * @param new_twist New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(const geometry_msgs::msg::Twist& old_twist, const geometry_msgs::msg::Twist& new_twist)
{
  const auto old_linear_x = std::abs(old_twist.linear.x);
  const auto new_linear_x = std::abs(new_twist.linear.x);

  const auto old_angular_z = std::abs(old_twist.angular.z);
  const auto new_angular_z = std::abs(new_twist.angular.z);

  return (old_linear_x  < new_linear_x ) or
         (old_angular_z < new_angular_z);
}

namespace twist_mux
{

TwistMux::TwistMux(const std::string name): rclcpp::Node(name)
,period_(1000)
{
  RCLCPP_INFO(this->get_logger(),"start.....");
  this->declare_parameter<std::string>("config","twist_mux_config.yaml");
  /// Get topics and locks:
  velocity_hs_ = std::make_shared<velocity_topic_container>();
  lock_hs_     = std::make_shared<lock_topic_container>();

  /*
  rcl_params_t *params_st = rcl_yaml_node_struct_init(rcutils_get_default_allocator ());
  rcl_parse_yaml_file(impl_->param_file_.c_str(),params_st);
  std::unordered_map<std::string,std::vector<rclcpp::Parameter>> pars= rclcpp::parameter_map_from(params_st);
  */
  getTopicHandles("topics", *velocity_hs_);
  getTopicHandles("locks" , *lock_hs_ );
  //std::shared_ptr<rclcpp::Node> p= std::make_shared<rclcpp::Node>(name+"AAAA");
  diagnostics_ = std::make_shared<diagnostics_type>( this);
  status_      = std::make_shared<status_type>(this->now());
  status_->velocity_hs = velocity_hs_;
  status_->lock_hs     = lock_hs_;
  std::string vel_topic_name ;
  vel_topic_name += this->get_namespace();
  vel_topic_name +="/cmd_vel";

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(vel_topic_name,1); 
  diagnostics_timer_ = this->create_wall_timer(std::chrono::seconds(/*DIAGNOSTICS_PERIOD*/ 1), std::bind(&TwistMux::updateDiagnostics, this));
  /*
    diagnostics_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      period_,
      std::bind(&TwistMux::updateDiagnostics, this));
   */   
}

TwistMux::~TwistMux()
{}

void TwistMux::updateDiagnostics()
{

  status_ ->last_loop_update     = this->now();
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

void TwistMux::publishTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
  geometry_msgs::msg::TwistStamped::SharedPtr tmsg = std::make_shared<geometry_msgs::msg::TwistStamped>(); 
  tmsg->header.frame_id="";
  tmsg->header.stamp = this->now();
  tmsg->twist.angular= msg->angular;
  tmsg->twist.linear = msg->linear;

  cmd_pub_->publish(*tmsg);
}

template<typename T>
void TwistMux::getTopicHandles(const std::string& param_name, std::list<T>& topic_hs)
{

   std::string config= this->get_parameter("config").as_string();
   
   RCLCPP_ERROR(this->get_logger(),"config name : %s for %s",config.c_str(),param_name.c_str());
   YAML::Node root_node = YAML::LoadFile(config);
   std::map<std::string,twist_mux::ParmsHolder> params;

   for (const auto& p : root_node[param_name]) {
      //RCLCPP_ERROR(this->get_logger(),"node %s is map {%d} ",param_name.c_str(),p.IsMap() );
    
    ParmsHolder ph;
    // Here 'p' is a map node, not a pair.
    for (const auto& key_value : p) {
      
      // Now 'key_value' is a key/value pair, so you can read it:
      YAML::Node key = key_value.first;
      YAML::Node value = key_value.second;
      std::string s = key.as<std::string>();
      

      if(s=="name"){
        ph.name= value.as<std::string>();
        
      }
      if(s=="topic"){
        ph.topic= value.as<std::string>();
      }
      if(s=="timeout"){
        ph.timeout= value.as<double>();
      }
      if(s=="priority"){
        ph.priority= value.as<int>();
      }
      //RCLCPP_WARN(this->get_logger(),"[%s] : %s",s.c_str(), value.as<std::string>().c_str());
     } 
     params.emplace(std::make_pair(ph.name,ph));
    }


    for(auto hh:params){
        topic_hs.emplace_back( hh.first, hh.second.topic, hh.second.timeout, hh.second.priority, this);
    }

   /* 
   root_node[param_name].as< 
   for (auto yaml : root_node) {
      auto controller_name = yaml.first.as<std::string>();

    for (auto yaml : root_node) {
    }
  */
  /*
  try
  {

    xh::Array output;
    xh::fetchParam(this->get_node(), param_name, output);

    xh::Struct output_i;
    std::string name, topic;
    double timeout;
    int priority;
    for (int i = 0; i < output.size(); ++i)
    {
      xh::getArrayItem(output, i, output_i);

      xh::getStructMember(output_i, "name"    , name    );
      xh::getStructMember(output_i, "topic"   , topic   );
      xh::getStructMember(output_i, "timeout" , timeout );
      xh::getStructMember(output_i, "priority", priority);

      topic_hs.emplace_back(this->get_node(), name, topic, timeout, priority, this);
    }
  }
  catch (const xh::XmlrpcHelperException& e)
  {
    RCLCPP_FATAL_STREAM(this->get_logger(),"Error parsing params: "<< e->what());
  }
  */
 //TODO
 //topic_hs.emplace_back( param_name, "topic/"+param_name, 100, 10, this);
}

int TwistMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto& lock_h : *lock_hs_)
  {
    if (lock_h.isLocked())
    {
      auto tmp = lock_h.getPriority();
      if (priority < tmp)
      {
        priority = tmp;
      }
    }
  }

  RCLCPP_DEBUG(this->get_logger() , "Priority = %d",static_cast<int>(priority));

  return priority;
}

bool TwistMux::hasPriority(const VelocityTopicHandle& twist)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string velocity_name = "NULL";

  /// max_element on the priority of velocity topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto& velocity_h : *velocity_hs_)
  {
    if (not velocity_h.isMasked(lock_priority))
    {
      const auto velocity_priority = velocity_h.getPriority();
      if (priority < velocity_priority)
      {
        priority = velocity_priority;
        velocity_name = velocity_h.getName();
      }
    }
  }

  return twist.getName() == velocity_name;
}

} // namespace twist_mux
