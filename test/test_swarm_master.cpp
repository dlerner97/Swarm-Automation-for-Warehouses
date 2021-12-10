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
#include <algorithm>
#include <gtest/gtest.h>
#include "../include/swarm_master/swarm_master.hpp"

TEST(SwarmMasterTests, TestAddRobotToSwarm) {
    SwarmMaster master;
    std::vector<int> robs_ids{};
    std::array<double, 2> initial_pos{2, 3};
    robs_ids.push_back(master.add_robot_to_swarm(initial_pos));
    robs_ids.push_back(master.add_robot_to_swarm({1,4}));
    const auto& avail_robots = master.get_avail_robots();
    for (const auto& id_robot : robs_ids)
        EXPECT_EQ(id_robot, avail_robots.at(id_robot).id);
    
    EXPECT_EQ(avail_robots.at(robs_ids[0]).pos[0], 2);
    EXPECT_EQ(avail_robots.at(robs_ids[0]).pos[1], 3);
    EXPECT_EQ(avail_robots.at(robs_ids[1]).pos[0], 1);
    EXPECT_EQ(avail_robots.at(robs_ids[1]).pos[1], 4);
}

TEST(SwarmMasterTests, TestAssignCrates) {
    SwarmMaster master(2.0);
    std::vector<int> site_ids{};
    Crate c1({1,2,3}, {4,5,6}, {3,4}, 6.2);
    site_ids.push_back(master.add_crate_to_system(c1));
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {4,6}, 1}));
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {4,6}, 5}));
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {4,6}, 7.2}));
    const auto& sites = master.get_sites();
    for (const auto& id_site : site_ids)
        EXPECT_EQ(id_site, sites.at(id_site).site_id);

    EXPECT_EQ(sites.at(site_ids[0]).crate.goal_pos, c1.goal_pos);
    EXPECT_EQ(sites.at(site_ids[1]).site_id, 1);
    EXPECT_TRUE(sites.at(site_ids[2]).assigned_ids.empty());
    EXPECT_EQ(sites.at(site_ids[0]).robots_required, 4);
    EXPECT_EQ(sites.at(site_ids[1]).robots_required, 2);
    EXPECT_EQ(sites.at(site_ids[2]).robots_required, 3);
    EXPECT_EQ(sites.at(site_ids[3]).robots_required, 4);
    EXPECT_ANY_THROW(master.add_crate_to_system({{3,2,1}, {6,5,4}, {4,6}, 8.1}));
}

// TEST(SwarmMasterTest, TestResetSwarm) {
//     SwarmMaster master(2.0);
//     master.add_robot_to_swarm({1,4});
//     master.add_crate_to_system({{3,2,1}, {6,5,4}, {4,6}, 1});

// }

TEST(SwarmMasterTest, TestNotEnoughRobots) {
    SwarmMaster master(2.0);
    master.add_crate_to_system({{1,2,3}, {4,5,6}, {3,4}, 5.8});
    master.add_robot_to_swarm({1,4});
    EXPECT_FALSE(master.enough_robots_for_assignments());
    EXPECT_EQ(master.assign_robots_to_crates()->size(), 0);
}

TEST(SwarmMasterTest, TestSwarmOccupied) {
    SwarmMaster master(2.0);
    master.add_crate_to_system({{1,2,3}, {4,5,6}, {3,4}, 1});
    master.add_robot_to_swarm({1,4});
    master.add_robot_to_swarm({2,5});
    EXPECT_TRUE(master.enough_robots_for_assignments());
    EXPECT_NE(master.assign_robots_to_crates()->size(), 0);
    EXPECT_TRUE(master.enough_robots_for_assignments());
    EXPECT_EQ(master.assign_robots_to_crates()->size(), 0);
    master.reset_swarm();
    EXPECT_NE(master.assign_robots_to_crates()->size(), 0);
}

// TEST(SwarmMasterTest, TestAssignRobotsToCrates) {
//     SwarmMaster master(2.0);
//     master.add_crate_to_system({{1,2,3}, {4,5,6}, {3,4}, 5.8});
//     master.add_crate_to_system({{3,2,1}, {6,5,4}, {6,4}, 1});
//     master.add_robot_to_swarm({1,4});
//     master.add_robot_to_swarm({3,2});
//     master.add_robot_to_swarm({2,3});
//     master.add_robot_to_swarm({5,5});
//     master.add_robot_to_swarm({10,10});

//     const auto& sites = master.get_sites();
//     auto assignments = master.assign_robots_to_crates();
//     int s1 = 0, s2 = 0;
//     std::vector<int> all_robot_ids{};
//     std::vector<std::array<double, 2> > pos_along_crate_s1{};
//     std::vector<std::array<double, 2> > pos_along_crate_s2{};
//     for (const auto& assignment : assignments) {
//         if (assignment.site_id == 0) {
//             s1++;
//             pos_along_crate_s1.push_back(assignment.pos_crate_frame);
//         }
//         else if (assignment.site_id == 1) {
//             s2++;
//             pos_along_crate_s2.push_back(assignment.pos_crate_frame);
//         }
//         all_robot_ids.push_back(assignment.robot_id);
//     }
//     EXPECT_EQ(s1, sites[0].robots_required);
//     EXPECT_EQ(s2, sites[1].robots_required);

//     std::vector<int>::iterator it;
//     const int size_all_robot_ids = all_robot_ids.size();

//     it = std::unique(all_robot_ids.begin(), all_robot_ids.end());
//     all_robot_ids.resize(std::distance(all_robot_ids.begin(),it));
//     EXPECT_EQ(all_robot_ids.size(), size_all_robot_ids);

//     EXPECT_TRUE(std::find(pos_along_crate_s1.begin(), pos_along_crate_s1.end(),
//         std::array<double, 2>({-2, 0})) != pos_along_crate_s1.end());
    
//     EXPECT_TRUE(std::find(pos_along_crate_s1.begin(), pos_along_crate_s1.end(),
//         std::array<double, 2>({1.5, 1.5})) != pos_along_crate_s1.end());
    
//     EXPECT_TRUE(std::find(pos_along_crate_s1.begin(), pos_along_crate_s1.end(),
//         std::array<double, 2>({1.5, -1.5})) != pos_along_crate_s1.end());

//     EXPECT_TRUE(std::find(pos_along_crate_s2.begin(), pos_along_crate_s2.end(),
//         std::array<double, 2>({3, 0})) != pos_along_crate_s2.end());

//     EXPECT_TRUE(std::find(pos_along_crate_s2.begin(), pos_along_crate_s2.end(),
//         std::array<double, 2>({-3, 0})) != pos_along_crate_s2.end());
// }