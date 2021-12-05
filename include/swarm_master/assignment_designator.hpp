/**
 * @file AssignmentDesignator.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief AssignmentDesignator class declaration
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <array>
#include <vector>
#include <memory>
#include <unordered_map>

struct Robot {
    int id;
    std::array<double, 2> pos;
    bool assigned = false;
    Robot(int _id, std::array<double, 2>& _pos) {
        this->id = _id;
        this->pos = _pos;
    }
};

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

class AssignmentDesignator {
 protected:
    typedef std::shared_ptr<std::vector<Site>> SiteVec;
    std::unordered_map<int, Site> all_sites;
    std::unordered_map<int, Robot> all_robots;
 public:
    AssignmentDesignator(std::vector<Site>& sites, std::vector<Robot> robots) {
        for (const auto& robot : robots) {
            all_robots[robot.id] = robot;
        }

        for (const auto& site : sites) {
            all_sites[site.site_id] = site;
        }
    }
    ~AssignmentDesignator() {}

    virtual SiteVec get_designations() = 0;
};

class GrowingRadiusDesignator : public AssignmentDesignator {
 public:
    GrowingRadiusDesignator(std::vector<Site>& sites, std::vector<Robot> robots) :
        AssignmentDesignator{sites, robots} {}

    SiteVec get_designations();
};