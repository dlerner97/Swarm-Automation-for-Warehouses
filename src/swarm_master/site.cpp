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
        double dist_sq = pow(pos[0], robot.second.pos[0]) + pow(pos[1], robot.second.pos[1]);
        dist_to_robots.push_back({robot.first, dist_sq});
    }
    std::sort(dist_to_robots.begin(), dist_to_robots.end());
}

std::vector<RobotDist> Site::get_n_closest(int n) {
    return std::vector<RobotDist>(dist_to_robots.begin(), dist_to_robots.end()+n);
}

std::vector<RobotDist> Site::find_all_lt_dist(double dist) {
    int index = 0;
    for (std::size_t i=0; i < dist_to_robots.size(); i++) {
        if (dist_to_robots[i].dist_sq > dist) break;
        index = i;
    }
    return get_n_closest(index);
}