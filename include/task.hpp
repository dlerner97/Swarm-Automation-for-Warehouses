/**
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Task struct declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#pragma once

#include <string>
#include <unordered_map>

struct Task {
    enum taskType {
        Drive,
        MvPlatform,
    };

    taskType task;
    std::unordered_map<std::string, double> num_param_dict{};
    std::unordered_map<std::string, std::string> command_param_dict{};
};
