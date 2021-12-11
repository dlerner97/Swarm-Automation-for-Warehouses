/**
 * @file assignment_designator.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief AssignmentDesignator executable class
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#include <map>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <unordered_map>
#include "../../include/swarm_master/assignment_designator.hpp"

SimpleClosestDesignator::SiteVec SimpleClosestDesignator::get_designations(
        std::unordered_map<int, Site>& all_sites,
        std::unordered_map<int, Robot>& all_robots) {
    SiteVec ret = std::make_shared<std::vector<Site> >();
    int robots_required{0};
    std::unordered_map<int, int> used_id_map{};
    for (auto& site_pair : all_sites) {
        auto& site = site_pair.second;
        site.populate_robot_dists(all_robots);
        for (const auto& robot : site.get_n_closest(-1)) {
            if (used_id_map.find(robot.robot_id) != used_id_map.end()) continue;
            if (site.assigned_ids.size() == site.robots_required) break;
            used_id_map[robot.robot_id] = site.site_id;
            site.assigned_ids.push_back(robot.robot_id);
        }
        if (site.assigned_ids.size() != site.robots_required)
            throw std::length_error("Error finding robot assignments");
    }

    for (const auto& used_id : used_id_map) {
        auto& site = all_sites.at(used_id.second);
        if (site.assigned_ids.size() < site.robots_required) {
            all_sites.at(used_id.second).assigned_ids.push_back(used_id.first);
        }
    }
    
    for (auto site : all_sites)
        ret->push_back(site.second);

    return ret;
}