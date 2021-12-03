/**
 * @file RosSwarmRobot.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief RosSwarmRobot class declaration
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
#include "./swarm_robot.hpp"

class RosSwarmRobot : public SwarmRobot {
 private:
    std::string task_service_topic;
    std::string pos_publisher_topic;
    ros::ServiceClient swarm_connect_client;
    ros::ServiceServer task_server;
    ros::Publisher pos_publisher;

    void get_task_callback(/* TBD */);
 public:
    RosSwarmRobot(/* args */);
    ~RosSwarmRobot();

    /**
     * @brief Connect to swarm master
     * 
     * @return true 
     * @return false 
     */
    bool connect_to_master();

    /**
     * @brief Publish robot position
     * 
     * @return true 
     * @return false 
     */
    void publish_robot_pos();

    /**
     * @brief Drive with mecanum wheels
     * 
     * @return true 
     * @return false 
     */
    bool drive_mecanum(std::array<double, 2>);

    /**
     * @brief Turn with mecanum wheels
     * 
     * @return true 
     * @return false 
     */
    bool turn_mecanum(double);

    /**
     * @brief Set the platform height
     * 
     * @return true 
     * @return false 
     */
    bool set_platform_height(double);
};
