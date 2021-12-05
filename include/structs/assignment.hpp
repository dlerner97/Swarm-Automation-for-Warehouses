/**
 * @file assignment.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Assignment struct declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include "./crate.hpp"
#include <array>

struct Assignment {
    int robot_id;
    Crate crate;
    std::array<double, 2> pos_crate_frame;  // m
};
