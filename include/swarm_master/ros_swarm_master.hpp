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
#include <string>
#include <array>
#include <vector>
#include <unordered_map>

#include "warehouse_swarm/SwarmConnect.h"
#include "./swarm_master.hpp"

class RosSwarmMaster : public SwarmMaster {
 private:
   ros::NodeHandle nh;
   ros::ServiceServer swarm_connect_server;
   ros::ServiceServer swarm_task_server;

   std::string robot_namespace_begin;
   std::string task_server_name;
   std::string swarm_connect_server_topic_name;

   std::unordered_map<int, ros::Subscriber> all_pos_subscriber;
   std::unordered_map<int, ros::Publisher> all_task_publisher;
   std::unordered_map<int, ros::Publisher> all_site_ready_pub;
   std::unordered_map<int, ros::Subscriber> robot_site_waiting_pub;

   /**
    * @brief Get task callback function for swarm task service server
    * 
    */
   void get_task_callback(/* Not sure what this is yet. */);

   /**
    * @brief Swarm connect callback function for swarm connect service server
    * 
    */
   bool swarm_connect_callback(warehouse_swarm::SwarmConnect::Request& req,
                               warehouse_swarm::SwarmConnect::Response& resp);

   /**
    * @brief Robot waiting topic callback
    * 
    * @param robot_id 
    * @return true 
    * @return false 
    */
   bool get_robot_waiting_callback(std_msgs::UInt16::ConstPtr& robot_id);

 public:
   RosSwarmMaster(/* args */) :
         swarm_connect_server_topic_name{"/swarm_connect"},
         robot_namespace_begin{"/robot_"},
         task_server_name{"/task"} {
            
      swarm_connect_server = nh.advertiseService(
         swarm_connect_server_topic_name, &RosSwarmMaster::swarm_connect_callback, this);      
   }
   ~RosSwarmMaster() {}

   /**
    * @brief Call SwarmMaster assign_robots_to_crate
    * 
    * @return true
    * @return false 
    */
   bool assign_robots();
};
