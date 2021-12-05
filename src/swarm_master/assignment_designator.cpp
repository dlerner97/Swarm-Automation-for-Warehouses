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

GrowingRadiusDesignator::SiteVec GrowingRadiusDesignator::get_designations() {
    SiteVec ret = std::make_shared<std::vector<Site>>(std::vector<Site>{});
    int total_robots_required{0};
    std::map<int, std::vector<int>> used_id_map;
    std::map<int, std::vector<RobotDist>> closest{};

    int additive = 0;
    bool first_iter{true};
    while ((used_id_map.size() < total_robots_required) || first_iter) {
        closest.clear();
        used_id_map.clear();

        for (auto& site_pair : all_sites) {
            auto& site = site_pair.second;
            site.populate_robot_dists(all_robots);
            auto closest_few = site.get_n_closest(site.robots_required + additive);
            for (auto& close : closest_few)
                    used_id_map[close.robot_id].push_back(site.site_id);

            closest[site.site_id] = closest_few;
            if (first_iter)
                total_robots_required += site.robots_required;
        }

        additive++;

        if (first_iter && (all_robots.size() < total_robots_required))
            throw std::invalid_argument("Not enough robots to complete the task!");

        first_iter = false;
    }

    for (const auto& used_id : used_id_map) {
        for (const auto& site_id : used_id.second) {
            auto& site = all_sites.at(site_id);
            if (site.assigned_ids.size() < site.robots_required)
                all_sites.at(site_id).assigned_ids.push_back(used_id.first);
        }
    }
    
    for (auto site : all_sites)
        ret->push_back(site.second);

    return ret;
}