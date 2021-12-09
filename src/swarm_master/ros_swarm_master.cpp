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

#include <stdexcept>
#include <string>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include "warehouse_swarm/RobotTask.h"
#include "../../include/swarm_master/ros_swarm_master.hpp"

bool RosSwarmMaster::swarm_connect_callback(
        warehouse_swarm::SwarmConnect::Request& req,
        warehouse_swarm::SwarmConnect::Response& resp) {
    resp.id = add_robot_to_swarm({req.x, req.y});
    all_task_publisher[resp.id] = nh.advertise<warehouse_swarm::RobotTask>(
        robot_namespace_begin + std::to_string(resp.id) + task_server_name, 10);
    return true;
}

bool RosSwarmMaster::get_robot_waiting_callback(std_msgs::UInt16::ConstPtr& robot_id) {
    auto all_waiting_AND_site = all_robots_at_site_waiting(robot_id->data);
    if (all_waiting_AND_site.first)
        all_site_ready_pub.at(all_waiting_AND_site.second).publish(std_msgs::Empty());
}

bool RosSwarmMaster::assign_robots() {
    auto assignments = assign_robots_to_crates();
    for (const auto& assignment : *assignments) {
        auto robot_tasks = break_down_assignment(assignment);
        for (const auto& task : *robot_tasks) {
            warehouse_swarm::RobotTask task_msg;
            if (task.task == task.Drive) {
                task_msg.taskType = task_msg.DRIVE;
                task_msg.first_param = task.num_param_dict.at("ToX");
                task_msg.second_param = task.num_param_dict.at("ToY");
                task_msg.third_param = task.num_param_dict.at("ToTheta");
            } else if (task.task == task.MvPlatform) {
                task_msg.taskType = task_msg.MVPLATFORM;
                task_msg.first_param = task.num_param_dict.at("PlatformHeight");
                task_msg.second_param = -1;
                task_msg.third_param = -1;
            } else if (task.task == task.Wait) {
                task_msg.taskType = task_msg.WAIT;
                task_msg.first_param = task.num_param_dict.at("AssignmentID");
                task_msg.second_param = -1;
                task_msg.third_param = -1;
            } else {
                throw std::invalid_argument("Unhandled task type!");
            }
            all_task_publisher.at(assignment.robot_id).publish(task_msg);
        }
    }
}