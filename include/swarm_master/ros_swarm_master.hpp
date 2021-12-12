/**
 * @file RosSwarmMaster.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief RosSwarmMaster class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include <string>
#include <array>
#include <vector>
#include <unordered_map>

#include "warehouse_swarm/Crate.h"
#include "warehouse_swarm/SwarmConnect.h"
#include "./swarm_master.hpp"

class RosSwarmMaster {
 private:
  SwarmMaster master;  
  ros::NodeHandle nh;
  ros::ServiceServer swarm_reset_server;
  ros::ServiceServer swarm_connect_server;
  ros::Subscriber swarm_task_subscriber;
  std::unordered_map<int, ros::Publisher> all_task_publisher;
  std::unordered_map<int, ros::Publisher> all_site_ready_pub;
  std::unordered_map<int, ros::Subscriber> robot_site_waiting_pub;

  /**
    * @brief Get task callback function for swarm task service server
    * 
    */
  void get_task_callback(const warehouse_swarm::Crate::ConstPtr& crate);

  /**
    * @brief Swarm connect callback function for swarm connect service server
    * 
    */
  bool swarm_connect_callback(warehouse_swarm::SwarmConnect::Request& req,
                              warehouse_swarm::SwarmConnect::Response& resp);

  /**
    * @brief Swarm connect callback function for swarm connect service server
    * 
    */
  bool swarm_reset_callback(std_srvs::Empty::Request&,
                            std_srvs::Empty::Response&);

  /**
    * @brief Robot waiting topic callback
    * 
    * @param robot_id 
    */
  void get_robot_waiting_callback(const std_msgs::UInt16::ConstPtr& robot_id);

 public:
  RosSwarmMaster(AssignmentDesignator* designator, double _weight_per_robot=2.0) : 
          master{designator, _weight_per_robot} {
      ROS_INFO_STREAM("Spinning up RosSwarmMaster.");
      swarm_reset_server = nh.advertiseService(
        "swarm_reset", &RosSwarmMaster::swarm_reset_callback, this);
      swarm_connect_server = nh.advertiseService(
        "swarm_connect", &RosSwarmMaster::swarm_connect_callback, this);
      swarm_task_subscriber = nh.subscribe("payload_details", 100,
          &RosSwarmMaster::get_task_callback, this);
  }
  ~RosSwarmMaster() {
      ROS_INFO_STREAM("Shutting down RosSwarmMaster.");
  }

  /**
    * @brief Call SwarmMaster assign_robots_to_crate
    * 
    */
  bool assign_robots();

  /**
   * @brief Startup the swarm
   * 
   * @param duration 
   */
  void startup(double duration=10, double hz=20);

};
