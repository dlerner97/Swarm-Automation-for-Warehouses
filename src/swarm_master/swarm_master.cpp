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
#include <utility>
#include <stdexcept>
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

void SwarmMaster::assign_crate(Crate crate) {
    int id;
    if (assigned_site_id.empty()) id = 0;
    else id = *(assigned_site_id.end()-1) + 1;
    if (crate.mass > 4 * weight_per_robot)
        throw std::invalid_argument("Crate is too heavy for system!");
    sites.push_back({id, crate, weight_per_robot});
    assigned_site_id.push_back(id);
}

std::vector<std::array<double, 2>> SwarmMaster::assign_robots_along_crate(const Site& site) {
    int num_robots = site.assigned_ids.size();
    std::vector<std::array<double, 2>> ret;
    ret.reserve(num_robots);
    auto footprint = site.crate.base_footprint;
    auto bigger_half_footprint = (footprint[0] > footprint[1] ? footprint[0]/2.0 : footprint[1]/2.0);
    auto smaller_half_footprint = (footprint[0] < footprint[1] ? footprint[0]/2.0 : footprint[1]/2.0);

    if (num_robots == 2) {
        ret.push_back({bigger_half_footprint, 0});
        ret.push_back({-bigger_half_footprint, 0});
    } else if (num_robots == 3) {
        ret.push_back({bigger_half_footprint-0.5, smaller_half_footprint});
        ret.push_back({bigger_half_footprint-0.5, -smaller_half_footprint});
        ret.push_back({-bigger_half_footprint, 0});
    } else if (num_robots == 4) {
        ret.push_back({bigger_half_footprint, 0});
        ret.push_back({-bigger_half_footprint, 0});
        ret.push_back({0, smaller_half_footprint});
        ret.push_back({0, -smaller_half_footprint});
    } else {
        throw std::invalid_argument("There must be fewer than 4 robots. We cannot lift this crate!");
    }
    return ret;
}

std::vector<Assignment> SwarmMaster::assign_robots_to_crates() {
    GrowingRadiusDesignator designator(sites, robots_avail);
    auto designations = designator.get_designations();
    std::vector<Assignment> assignments;
    for (const auto& designation : *designations) {
        auto along_crate = assign_robots_along_crate(designation);
        for (std::size_t i=0; i<designation.assigned_ids.size(); i++)            
            assignments.push_back({designation.assigned_ids[i], designation.crate, along_crate[i]});
    }
    return assignments;
}

const std::pair<std::vector<int>, std::vector<Robot> > SwarmMaster::get_avail_robots() {
    return std::make_pair(assigned_ids, robots_avail);
}

const std::vector<Site>& SwarmMaster::get_sites() {
    return sites;
}