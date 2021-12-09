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

#include <string>
#include <memory>
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
    robots_avail.emplace(id, Robot(id, pos_init));
    return id;
}

int SwarmMaster::add_crate_to_system(Crate crate) {
    int id;
    if (assigned_site_id.empty()) id = 0;
    else id = *(assigned_site_id.end()-1) + 1;
    if (crate.mass > 4 * weight_per_robot)
        throw std::invalid_argument("Crate is too heavy for system!");
    sites.emplace(id, Site(id, crate, weight_per_robot));
    required_robots_system += sites.at(id).robots_required;
    robots_at_site_waiting.insert({id, {}});
    assigned_site_id.push_back(id);
    return id;
}

bool SwarmMaster::enough_robots_for_assignments() {
    return robots_avail.size() >= required_robots_system;
}

std::vector<std::array<double, 3> > SwarmMaster::assign_robots_along_crate(const Site& site) {
    int num_robots = site.assigned_ids.size();
    std::vector<std::array<double, 3>> ret;
    ret.reserve(num_robots);
    auto footprint = site.crate.base_footprint;
    auto bigger_half_footprint = (footprint[0] > footprint[1] ? footprint[0]/2.0 : footprint[1]/2.0);
    auto smaller_half_footprint = (footprint[0] < footprint[1] ? footprint[0]/2.0 : footprint[1]/2.0);

    if (num_robots == 2) {
        ret.push_back({bigger_half_footprint, 0, 180});
        ret.push_back({-bigger_half_footprint, 0, 0});
    } else if (num_robots == 3) {
        ret.push_back({bigger_half_footprint-0.5, smaller_half_footprint, 270});
        ret.push_back({bigger_half_footprint-0.5, -smaller_half_footprint, 90});
        ret.push_back({-bigger_half_footprint, 0, 0});
    } else if (num_robots == 4) {
        ret.push_back({bigger_half_footprint, 0, 180});
        ret.push_back({-bigger_half_footprint, 0, 0});
        ret.push_back({0, smaller_half_footprint, 270});
        ret.push_back({0, -smaller_half_footprint, 90});
    } else {
        throw std::invalid_argument("There must be fewer than 4 robots. We cannot lift this crate!");
    }
    if (footprint[1] > footprint[0]) {
        for (auto& pos : ret) {
            auto temp = pos[0];
            pos[0] = pos[1];
            pos[1] = temp;
            pos[2] = (static_cast<int>(pos[2])+90)%360;
        }
    }

    return ret;
}

std::shared_ptr<std::vector<Assignment> > SwarmMaster::assign_robots_to_crates() {
    std::shared_ptr<std::vector<Assignment> > assignments;
    if (!swarm_is_occupied) {
        swarm_is_occupied = true;
        GrowingRadiusDesignator designator(sites, robots_avail);
        auto designations = designator.get_designations();
        for (const auto& designation : *designations) {
            auto along_crate = assign_robots_along_crate(designation);
            for (std::size_t i=0; i<designation.assigned_ids.size(); i++) {
                int robot_id = designation.assigned_ids[i];
                robots_avail.at(robot_id).assigned_site = designation.site_id;
                assignments->push_back({robot_id, designation.site_id,
                            designation.crate, along_crate[i]});
            }
        }
    }
    return assignments;
}

std::shared_ptr<std::vector<Task> > SwarmMaster::break_down_assignment(const Assignment& assignment) {
    std::shared_ptr<std::vector<Task>> ret;
    typedef std::unordered_map<std::string, double> commandDict;
    ret->push_back({Task::MvPlatform, commandDict{{"PlatformHeight", assignment.crate.start_pos[2]-0.5}}});

    auto toX_begin = assignment.crate.start_pos[0] + assignment.pos_crate_frame[0];
    auto toY_begin = assignment.crate.start_pos[1] + assignment.pos_crate_frame[1];

    ret->push_back({Task::Drive, commandDict{{"ToX", toX_begin}, {"ToY", toY_begin}, {"ToTheta", assignment.pos_crate_frame[2]}}});
    ret->push_back({Task::MvPlatform, commandDict{{"PlatformHeight", assignment.crate.start_pos[2]}}});
    ret->push_back({Task::Wait, commandDict{{"AssignmentID", assignment.site_id}}});

    auto toX_end = assignment.crate.goal_pos[0] + assignment.pos_crate_frame[0];
    auto toY_end = assignment.crate.goal_pos[1] + assignment.pos_crate_frame[1];
    ret->push_back({Task::Drive, commandDict{{"ToX", toX_end}, {"ToY", toY_end}, {"ToTheta", assignment.pos_crate_frame[2]}}});
    ret->push_back({Task::Wait, commandDict{{"AssignmentID", assignment.site_id}}});
    ret->push_back({Task::MvPlatform, commandDict{{"PlatformHeight", assignment.crate.start_pos[2]-0.5}}});
    return ret;
}

std::pair<bool, int> SwarmMaster::all_robots_at_site_waiting(int robot_id) {
    int site_id = robots_avail.at(robot_id).assigned_site;
    robots_at_site_waiting.at(site_id).insert(robot_id);
    if (robots_at_site_waiting.at(site_id).size() == sites.at(site_id).robots_required) {
        robots_at_site_waiting.at(site_id).clear();
        return std::make_pair(true, site_id);
    }
    return std::make_pair(false, site_id);
}

void SwarmMaster::clear_crates() {
    sites.clear();
    assigned_site_id.clear();
}

void SwarmMaster::clear_robots() {
    robots_avail.clear();
    assigned_ids.clear();
}

void SwarmMaster::reset_swarm() {
    clear_crates();
    clear_robots();
    swarm_is_occupied = false;
}

const std::unordered_map<int, Robot> SwarmMaster::get_avail_robots() {
    return robots_avail;
}

const std::unordered_map<int, Site>& SwarmMaster::get_sites() {
    return sites;
}