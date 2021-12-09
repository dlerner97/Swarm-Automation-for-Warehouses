/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <vector>
#include "../../include/structs/crate.hpp"
#include "../../include/task_orchestrator/task_orchestrator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_orchestrator");
    std::vector<Crate> crates;
    crates.emplace_back(Crate({2,3,4}, {1,2,3}, {5,4}, 1));
    TaskOrchestrator taskOrch;
    taskOrch.publish_full_task_list(crates);
    return 0;
}