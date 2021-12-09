/**
 * @file TaskOrchestrator.hpp
 * @author Dani Lerner (you@domain.com)
 * @brief TaskOrchestrator class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <vector>
#include <ros.h>
#include <string>

#include "../structs/crate.hpp"

class TaskOrchestrator {
 private:
    std::vector<Crate> crates:
    ros::ServiceClient;
    std::string swarm_task_server_topic_name;
 public:
    TaskOrchestrator(/* args */) {}
    ~TaskOrchestrator() {}

    /**
     * @brief Get the full task list
     * 
     * @return std::vector<Crate> - full task list
     * 
     */
    std::vector<Crate> get_full_task(/* unknown ATM */);

    /**
     * @brief Publishes full task list to task service
     * 
     * @return true 
     * @return false 
     */
    bool publish_full_task_list(std::vector<Crate>& crates);
};

