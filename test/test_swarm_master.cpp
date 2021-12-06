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

#include <iostream>
#include <array>
#include <gtest/gtest.h>
#include "../include/swarm_master/swarm_master.hpp"

TEST(SwarmMasterTests, TestAddRobotToSwarm) {
    SwarmMaster master;
    std::array<double, 2> initial_pos{2, 3};
    master.add_robot_to_swarm(initial_pos);
    master.add_robot_to_swarm({1,4});
    const auto& id_robots = master.get_avail_robots();
    auto assigned_ids = id_robots.first;
    auto robots_avail = id_robots.second;
    EXPECT_EQ(assigned_ids[0], 0);
    EXPECT_EQ(assigned_ids[1], 1);
    EXPECT_EQ(robots_avail[0].id, 0);
    EXPECT_EQ(robots_avail[0].pos[0], 2);
    EXPECT_EQ(robots_avail[0].pos[1], 3);
    EXPECT_EQ(robots_avail[1].pos[0], 1);
    EXPECT_EQ(robots_avail[1].pos[1], 4);
}

TEST(SwarmMasterTests, TestAssignCrates) {
    SwarmMaster master(2.0);
    Crate c1({1,2,3}, {4,5,6}, {3,4}, 6.2);
    master.assign_crate(c1);
    master.assign_crate({{3,2,1}, {6,5,4}, {4,6}, 1});
    master.assign_crate({{3,2,1}, {6,5,4}, {4,6}, 5});
    master.assign_crate({{3,2,1}, {6,5,4}, {4,6}, 7.2});
    const auto& sites = master.get_sites();
    EXPECT_EQ(sites[0].crate.goal_pos, c1.goal_pos);
    EXPECT_EQ(sites[1].site_id, 1);
    EXPECT_TRUE(sites[2].assigned_ids.empty());
    EXPECT_EQ(sites[0].robots_required, 4);
    EXPECT_EQ(sites[1].robots_required, 2);
    EXPECT_EQ(sites[2].robots_required, 3);
    EXPECT_EQ(sites[3].robots_required, 4);
    EXPECT_ANY_THROW(master.assign_crate({{3,2,1}, {6,5,4}, {4,6}, 8.1}));
}

TEST(SwarmMasterTest, TestAssignRobotsToCrates) {
    SwarmMaster master(2.0);
    master.assign_crate({{1,2,3}, {4,5,6}, {3,4}, 6.2});
    master.add_robot_to_swarm({1,4});

    
}