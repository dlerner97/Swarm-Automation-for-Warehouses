/**
 * @file swarm_master.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief SwarmMaster class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <vector>
#include <queue>
#include "./site.hpp"
#include "../structs/task.hpp"
#include "../structs/crate.hpp"
#include "../structs/robot.hpp"
#include "../structs/assignment.hpp"
#include "./assignment_designator.hpp"

class SwarmMaster {
 protected:
   double weight_per_robot = 2;        // kg
   std::vector<int> assigned_ids{};
   std::vector<Robot> robots_avail{};
   std::vector<int> assigned_site_id{};
   std::vector<Site> sites{};
 public:
   SwarmMaster(/* args */) {}
   ~SwarmMaster() {}

   /**
    * @brief Add robot to swarm
    * 
    * @param robot 
    * @return int 
    */
   int add_robot_to_swarm(std::array<double, 2>);

   /**
    * @brief Add task to task queue
    * 
    */
   void assign_crates(std::vector<Crate>);

   /**
    * @brief Find the relative positions of each robot in the crate frame
    * 
    * @param site 
    * @return std::vector<std::array<double, 2>> 
    */
   std::vector<std::array<double, 2>> assign_robots_along_crate(const Site& site);

   /**
    * @brief Assign all robots to designated crates
    * 
    * @return std::vector<Assignment> 
    */
   std::vector<Assignment> assign_robots_to_crates();

   /**
    * @brief Grab next task from task queue and get it done.
    * 
    * @return true 
    * @return false 
    */
   bool perform_next_task();
};
