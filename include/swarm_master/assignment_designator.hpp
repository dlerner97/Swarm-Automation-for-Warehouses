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
#include "../structs/robot.hpp"
#include "./site.hpp"

class AssignmentDesignator {
 protected:
    typedef std::shared_ptr<std::vector<Site>> SiteVec;
    std::unordered_map<int, Site> all_sites;
    std::unordered_map<int, Robot> all_robots;
 public:
    AssignmentDesignator(std::vector<Site>& sites, std::vector<Robot>& robots) {
        for (const auto& robot : robots) {
            all_robots.emplace(robot.id, robot);
        }

        for (const auto& site : sites) {
            all_sites.emplace(site.site_id, site);
        }
    }
    ~AssignmentDesignator() {}

    virtual SiteVec get_designations() = 0;
};

class GrowingRadiusDesignator : public AssignmentDesignator {
 public:
    GrowingRadiusDesignator(std::vector<Site>& sites, std::vector<Robot>& robots) :
        AssignmentDesignator{sites, robots} {}

    SiteVec get_designations();
};