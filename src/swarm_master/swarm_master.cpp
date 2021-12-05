/**
 * @file swarm_master.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief SwarmMaster class executable
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#include <vector>
#include "../../include/swarm_master/swarm_master.hpp"
#include "../../include/swarm_master/assignment_designator.hpp"

int SwarmMaster::add_robot_to_swarm(std::array<double, 2> pos_init) {
    int id;
    if (assigned_ids.empty()) id = 0;
    else id = *(assigned_ids.end()-1) + 1;
    assigned_ids.push_back(id);
    robots_avail.push_back({id, pos_init});
    return id;
}