/**
 * @file test_swarm_master.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Tests for the swarm master
 * @version 0.1
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#include <array>
#include <gtest/gtest.h>
#include "../include/swarm_master/swarm_master.hpp"

TEST(SwarmMasterTests, TestAddRobotToSwarm) {
    SwarmMaster master;
    std::array<double, 2> initial_pos{2, 3};
    master.add_robot_to_swarm(initial_pos);
    master.add_robot_to_swarm({1,4});
    auto id_robots = master.get_avail_robots();
    auto assigned_ids = id_robots.first;
    auto robots_avail = id_robots.second;
    EXPECT_EQ(assigned_ids[0], 0);
    EXPECT_EQ(assigned_ids[1], 1);
    EXPECT_EQ(robots_avail[0].id, 0);
    EXPECT_EQ(robots_avail[0].pos[0], 2);
    EXPECT_EQ(robots_avail[0].pos[1], 3);
}