/**
 * @file ros_swarm_master.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Ros Swarm Master class exe
 * @version 0.1
 * @date 2021-12-06
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#include <string>
#include <ros/ros.h>
#include "warehouse_swarm/RobotTask.h"
#include "../../include/swarm_master/ros_swarm_master.hpp"

bool RosSwarmMaster::swarm_connect_callback(
        warehouse_swarm::SwarmConnect::Request& req,
        warehouse_swarm::SwarmConnect::Response& resp) {
    resp.id = add_robot_to_swarm({req.x, req.y});
    all_task_publisher[resp.id] = nh.advertise<warehouse_swarm::RobotTask>(
        robot_namespace_begin + std::to_string(resp.id) + task_server_name, 10);
}