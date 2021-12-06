/**
 * @file crate.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Crate struct declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <array>

struct Crate {
    double mass;                              // kg
    std::array<double, 3> start_pos;          // m
    std::array<double, 3> goal_pos;           // m
    std::array<double, 2> base_footprint;     // m

    Crate() : start_pos{}, goal_pos{}, base_footprint{}, mass{} {}

    Crate(std::array<double, 3> _start_pos, std::array<double, 3> _goal_pos,
            std::array<double, 2> _base_footprint, double _mass) :
        mass{_mass},
        start_pos{_start_pos},
        goal_pos{_goal_pos},
        base_footprint{_base_footprint} {}
};
