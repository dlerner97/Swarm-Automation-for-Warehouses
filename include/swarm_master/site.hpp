/**
 * @file site.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <array>
#include <vector>
#include <unordered_map>
#include "../structs/robot.hpp"

struct RobotDist {
    int robot_id;
    double dist_sq;
    RobotDist(int _id, double _dist) {
        this->robot_id = _id;
        this->dist_sq = _dist;
    }
};

bool operator<(const RobotDist& d1, const RobotDist& d2) {
    return d1.dist_sq < d2.dist_sq;
}

class Site {
 public:
    int site_id;
    int robots_required;
    std::array<double, 2> pos;
    std::vector<int> assigned_ids{};
    std::vector<RobotDist> dist_to_robots{};

    Site(int _site_id, int _robots_req, std::array<double, 2>& _pos) {
        this->site_id = _site_id;
        this->robots_required = _robots_req;
        this->pos = _pos;
    }
    void populate_robot_dists(std::unordered_map<int, Robot>& robots);

    std::vector<RobotDist> get_n_closest(int n);

    std::vector<RobotDist> find_all_lt_dist(double dist);
};