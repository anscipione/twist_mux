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

#include "rclcpp/rclcpp.hpp"
#include <twist_mux/twist_mux.h>

int
main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

/*
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto talker = std::make_shared<twist_mux::TwistMux>(options);
  exec.add_node(talker);

  exec.spin();
*/
  rclcpp::executors::SingleThreadedExecutor exec;
  auto mux =std::make_shared<twist_mux::TwistMux>();
  exec.add_node(mux);
  exec.spin();
  RCLCPP_INFO(mux->get_logger(),"SPIN EXIT");
  //rclcpp::spin(std::make_shared<twist_mux::TwistMux>());

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

