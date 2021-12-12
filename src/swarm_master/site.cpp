/**
 * @file site.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Site class implementation
 * @version 0.1
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include "../../include/swarm_master/site.hpp"

void Site::populate_robot_dists(std::unordered_map<int, Robot>& robots) {
    for (const auto& robot : robots) {
        double dist_sq = pow(crate.start_pos[0], robot.second.pos[0]) + pow(crate.start_pos[1], robot.second.pos[1]);
        dist_to_robots.push_back({robot.first, dist_sq});
    }
    std::sort(dist_to_robots.begin(), dist_to_robots.end());
}

std::vector<RobotDist> Site::get_n_closest(int n) {
    if (n == -1)
        return std::vector<RobotDist>(dist_to_robots.begin(), dist_to_robots.end());
    else if (n >= dist_to_robots.size()) throw std::invalid_argument("Not enough robots");
    else
        return std::vector<RobotDist>(dist_to_robots.begin(), dist_to_robots.begin()+n);
}
