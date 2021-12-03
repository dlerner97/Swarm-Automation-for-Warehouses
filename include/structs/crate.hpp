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
    double mass{};                              // kg
    std::array<double, 3> start_pos{};          // m
    std::array<double, 3> goal_pos{};           // m
    std::array<double, 2> base_footprint{};     // m
};
