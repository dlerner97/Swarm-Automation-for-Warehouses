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
    typedef std::shared_ptr<std::vector<Site> > SiteVec;
 public:  
    virtual SiteVec get_designations(std::unordered_map<int, Site>& all_sites,
                                     std::unordered_map<int, Robot>& all_robots) = 0;
};

class SimpleClosestDesignator : public AssignmentDesignator {
 public:
    SiteVec get_designations(std::unordered_map<int, Site>& all_sites,
                             std::unordered_map<int, Robot>& all_robots);
};