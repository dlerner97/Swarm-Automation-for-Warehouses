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
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include "warehouse_swarm/Crate.h"
#include "warehouse_swarm/RobotTask.h"
#include "../../include/swarm_master/ros_swarm_master.hpp"

bool RosSwarmMaster::swarm_connect_callback(
        warehouse_swarm::SwarmConnect::Request& req,
        warehouse_swarm::SwarmConnect::Response& resp) {
    resp.id = master.add_robot_to_swarm({req.x, req.y});
    all_task_publisher[resp.id] = nh.advertise<warehouse_swarm::RobotTask>(
        "robot_" + std::to_string(resp.id) + "/task", 10);
    return true;
}

bool RosSwarmMaster::swarm_reset_callback(std_srvs::Empty::Request&,
                                          std_srvs::Empty::Response&) {
    ROS_INFO_STREAM("RESETTING SWARM");
    master.reset_swarm();
    return true;
}

void RosSwarmMaster::get_robot_waiting_callback(const std_msgs::UInt16::ConstPtr& robot_id) {
    auto all_waiting_AND_site = master.all_robots_at_site_waiting(robot_id->data);
    if (all_waiting_AND_site.first)
        all_site_ready_pub.at(all_waiting_AND_site.second).publish(std_msgs::Empty());
}

void RosSwarmMaster::get_task_callback(const warehouse_swarm::Crate::ConstPtr& crate) {
    int site_id = master.add_crate_to_system(
        Crate({crate->start_pos.x, crate->start_pos.y, crate->start_pos.z},
              {crate->goal_pos.x, crate->goal_pos.y, crate->goal_pos.z},
              {crate->x_len, crate->y_len}, crate->mass));

    const std::string prefix = "site_" + std::to_string(site_id);
    all_site_ready_pub[site_id] = nh.advertise<std_msgs::Empty>(prefix + "/ready", 1);
    robot_site_waiting_pub[site_id] = nh.subscribe(prefix + "/waiting", 10,
        &RosSwarmMaster::get_robot_waiting_callback, this);
}

bool RosSwarmMaster::assign_robots() {
    auto assignments = master.assign_robots_to_crates();
    for (const auto& assignment : *assignments) {
        auto robot_tasks = master.break_down_assignment(assignment);
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
    return !assignments->empty();
}

void RosSwarmMaster::startup(double duration, double hz) {
    ros::Duration dur(duration);
    ros::Rate rate(hz);

    bool assigned = false;
    while (!assigned) {
        auto startup_begin_time = ros::Time::now();
        while (ros::ok() && ros::Time::now() - startup_begin_time < dur) {
            ros::spinOnce();
            rate.sleep();
        }
        assigned = assign_robots();
    }

}