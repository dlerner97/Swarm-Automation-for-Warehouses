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

#include <math.h>
#include <array>
#include <vector>
#include <unordered_map>
#include "../structs/robot.hpp"
#include "../structs/crate.hpp"

struct RobotDist {
    int robot_id;
    double dist_sq;
    RobotDist(int _id, double _dist) {
        this->robot_id = _id;
        this->dist_sq = _dist;
    }
    bool operator<(const RobotDist& d2) {
        return this->dist_sq < d2.dist_sq;
    }
};

class Site {
 public:
    int site_id;
    Crate crate{};
    int robots_required;
    std::array<double, 2> pos;
    std::vector<int> assigned_ids{};
    std::vector<RobotDist> dist_to_robots{};

    Site(int _site_id, int _robots_req, std::array<double, 2>& _pos):
        site_id{_site_id},
        robots_required{_robots_req},
        pos{_pos} {}

    Site(int id, const Crate& _crate, double weight_per_robot) :
            site_id{id},
            pos{_crate.base_footprint[0], _crate.base_footprint[1]} {
        double val = std::ceil(_crate.mass/weight_per_robot);
        if (val < 2) val = 2;
        robots_required = static_cast<int>(val);
        crate = _crate;
    }

    Site(const Site& site) {
        this->assigned_ids = site.assigned_ids;
        this->crate = site.crate;
        this->dist_to_robots = site.dist_to_robots;
        this->pos = site.pos;
        this->site_id = site.site_id;
        this->robots_required = site.robots_required;
    }

    Site& operator=(Site& site) {
        return site;
    }

    void populate_robot_dists(std::unordered_map<int, Robot>& robots);

    std::vector<RobotDist> get_n_closest(int n);

    std::vector<RobotDist> find_all_lt_dist(double dist);
};