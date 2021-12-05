/**
 * @file swarm_master.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief SwarmMaster class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <vector>
#include <queue>
#include "./site.hpp"
#include "../structs/task.hpp"
#include "../structs/crate.hpp"
#include "../structs/robot.hpp"
#include "../structs/assignment.hpp"
#include "./assignment_designator.hpp"

class SwarmMaster {
 protected:
    std::vector<int> assigned_ids;
    std::vector<Robot> robots_avail{};
    std::vector<Site> sites{};
    std::queue<Task> task_queue;
 public:
    SwarmMaster(/* args */) {}
    ~SwarmMaster() {}

    /**
     * @brief Add task to task queue
     * 
     */
    void add_to_task_queue(std::vector<Crate>);

    /**
     * @brief Assign all robots to designated crates
     * 
     * @return std::vector<Assignment> 
     */
    std::vector<Assignment> assign_robots_to_crates();

    /**
     * @brief Grab next task from task queue and get it done.
     * 
     * @return true 
     * @return false 
     */
    bool perform_next_task();
};
