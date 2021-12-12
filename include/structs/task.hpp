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
        Wait,
    };

    taskType task;
    std::unordered_map<std::string, double> num_param_dict;

    Task(taskType task_type, std::unordered_map<std::string, double> command_dict) {
        task = task_type;
        num_param_dict = command_dict;
    }
};
