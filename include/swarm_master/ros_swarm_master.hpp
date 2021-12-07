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

#include <ros.h>
#include <string>
#include <array>
#include <vector>

#include "warehouse_swarm/SwarmConnect.h"
#include "./swarm_master.hpp"

class RosSwarmMaster : public SwarmMaster {
 private:
   ros::ServiceServer swarm_connect_server;
   ros::ServiceServer swarm_task_server;
   std::vector<ros::Subscriber> all_pos_subscriber;
   std::string swarm_task_server_topic_name;
   std::string swarm_connect_server_topic_name;

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

 public:
   RosSwarmMaster(/* args */) : swarm_connect_server_topic_name{"/swarm_connect"} {
      ros::NodeHandle nh;
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
