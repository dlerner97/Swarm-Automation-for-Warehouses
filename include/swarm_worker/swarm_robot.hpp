/**
 * @file SwarmRobot.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief SwarmRobot class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <queue>
#include <array>
#include "../structs/crate.hpp"
#include "../structs/task.hpp"

class SwarmRobot {
 protected:
    int robot_id;
    Crate designated_crate;
    std::queue<Task> task_queue;
    std::array<double, 3> curr_pos;

    /**
     * @brief Set the id int
     * 
     */
    void set_id(int);
 public:
    SwarmRobot(/* args */) {}
    ~SwarmRobot() {}

    /**
     * @brief Add task to task queue
     * 
     * @param task 
     */
    void add_to_task_queue(Task task);

    /**
     * @brief Get robot position in world
     * 
     * @return std::array<double, 3> - x, y, platform height
     */
    std::array<double, 3> get_pos();

    /**
     * @brief Drive to given waypoint with mecanum wheels
     * 
     * @param waypt3D
     * @return std::array<double, 2>
     */
    std::array<double, 2> mecanum_drive_to(std::array<double, 2> waypt3D);

    /**
     * @brief Turn towards given waypoint
     * 
     * @param waypt2D 
     * @return double 
     */
    double turn_towards(std::array<double, 2> waypt2D);

    /**
     * @brief Set the platform height
     * 
     * @param delta_h 
     * @return double 
     */
    double set_platform_height(double delta_h);
};
