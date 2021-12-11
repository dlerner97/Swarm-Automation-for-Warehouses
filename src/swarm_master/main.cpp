/**
 * @file main.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Main script for RosSwarmMaster
 * @version 0.1
 * @date 2021-12-06
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#include <ros/ros.h>
#include "../../include/swarm_master/ros_swarm_master.hpp"
#include "../../include/swarm_master/assignment_designator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_master");
    SimpleClosestDesignator designator;
    RosSwarmMaster master(&designator, 2.0);
    master.startup();
    ros::spin();
    return 0;
}