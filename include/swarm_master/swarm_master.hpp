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

#include <set>
#include <memory>
#include <vector>
#include <queue>
#include <utility>
#include <unordered_map>
#include "./site.hpp"
#include "../structs/task.hpp"
#include "../structs/crate.hpp"
#include "../structs/robot.hpp"
#include "../structs/assignment.hpp"
#include "./assignment_designator.hpp"

class SwarmMaster {
 protected:
  double weight_per_robot;            // kg
  std::vector<int> assigned_ids{};
  std::vector<int> assigned_site_id{};
  std::unordered_map<int, Robot> robots_avail{};
  std::unordered_map<int, Site> sites{};
  std::unordered_map<int, std::set<int>> robots_at_site_waiting;
 public:
  SwarmMaster(double _weight_per_robot=2.0) :
    weight_per_robot{_weight_per_robot} {}
  ~SwarmMaster() {}

  /**
  * @brief Add robot to swarm
  * 
  * @param robot 
  * @return int 
  */
  int add_robot_to_swarm(std::array<double, 2>);

  /**
  * @brief Assign crates as sites
  * 
  */
  int add_crate_to_system(Crate crate);

  /**
  * @brief Find the relative positions of each robot in the crate frame
  * 
  * @param site 
  * @return std::vector<std::array<double, 2>> 
  */
  std::vector<std::array<double, 3> > assign_robots_along_crate(const Site& site);

  /**
  * @brief Assign all robots to designated crates
  * 
  * @return std::vector<Assignment> 
  */
  std::shared_ptr<std::vector<Assignment> > assign_robots_to_crates();

  /**
   * @brief Break down assignment into list of tasks
   * 
   * @return std::vector<Task> 
   */
  std::shared_ptr<std::vector<Task> > break_down_assignment(const Assignment& assignment);

  /**
   * @brief Check if all robots have signaled that they are waiting
   * 
   * @param robot_id 
   * @param site_id 
   * @return true 
   * @return false 
   */
  std::pair<bool, int> all_robots_at_site_waiting(int robot_id);

  /**
  * @brief Grab next task from task queue and get it done.
  * 
  * @return true 
  * @return false 
  */
  bool perform_next_task();

  /**
   * @brief Clear crates from vect
   * 
   */
  void clear_crates();

  /**
   * @brief Clear robots from vec
   * 
   */
  void clear_robots();

  /**
  * @brief Get avail robots
  * 
  * @return std::pair<std::vector<int>, std::vector<Robot>> 
  */
  const std::unordered_map<int, Robot> get_avail_robots();

  /**
  * @brief Get assigned sites
  * 
  * @return const std::vector<Site>&
  */
  const std::unordered_map<int, Site>& get_sites();
};
