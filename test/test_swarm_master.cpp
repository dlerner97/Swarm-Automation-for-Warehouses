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
#include "../include/structs/task.hpp"
#include "../include/swarm_master/swarm_master.hpp"
#include "../include/swarm_master/assignment_designator.hpp"

bool operator==(const Task& t1, const Task& t2);
template<class Element>
bool found_element_in_vec(std::vector<Element>& pos_along_crate, Element pos);
std::vector<int> add_buncha_robots(SwarmMaster* master);

TEST(SwarmMasterTests, InitializeSwarmMaster) {
    SimpleClosestDesignator designator;
    EXPECT_NO_THROW(SwarmMaster master(&designator));
    EXPECT_NO_THROW(SwarmMaster master(&designator, 0.1));
    EXPECT_ANY_THROW(SwarmMaster master(&designator, 0.0));
}

TEST(SwarmMasterTests, TestAddRobotToSwarm) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
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
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
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

TEST(SwarmMasterTests, TestResetSwarm) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    master.add_robot_to_swarm({1,4});
    master.add_crate_to_system({{3,2,1}, {6,5,4}, {4,6}, 1});
    EXPECT_NE(master.get_avail_robots().size(), 0);
    EXPECT_NE(master.get_sites().size(), 0);

    master.reset_swarm();
    EXPECT_EQ(master.get_avail_robots().size(), 0);
    EXPECT_EQ(master.get_sites().size(), 0);
}

TEST(SwarmMasterTests, TestNotEnoughRobots) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    master.add_crate_to_system({{1,2,3}, {4,5,6}, {3,4}, 5.8});
    master.add_robot_to_swarm({1,4});
    EXPECT_FALSE(master.enough_robots_for_assignments());
    EXPECT_EQ(master.assign_robots_to_crates()->size(), 0);
}

TEST(SwarmMasterTests, TestSwarmOccupied) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    master.add_crate_to_system({{1,2,3}, {4,5,6}, {3,4}, 1});
    master.add_robot_to_swarm({1,4});
    master.add_robot_to_swarm({2,5});
    EXPECT_TRUE(master.enough_robots_for_assignments());
    EXPECT_NE(master.assign_robots_to_crates()->size(), 0);
    EXPECT_TRUE(master.enough_robots_for_assignments());
    EXPECT_EQ(master.assign_robots_to_crates()->size(), 0);

    master.clear_tasks();
    master.add_crate_to_system({{1,2,3}, {4,5,6}, {3,4}, 1});
    EXPECT_TRUE(master.enough_robots_for_assignments());
    EXPECT_NE(master.assign_robots_to_crates()->size(), 0);
}

TEST(SwarmMasterTests, TestAssignUniqueRobotsToMultipleCrates) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    std::vector<int> site_ids{};
    std::vector<int> robot_ids = add_buncha_robots(&master);

    site_ids.push_back(master.add_crate_to_system({{1,2,3}, {4,5,6}, {3,4}, 5.8}));
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {6,4}, 1}));
    site_ids.push_back(master.add_crate_to_system({{11,12,11}, {16,15,14}, {5,5}, 7.2}));

    const auto& sites = master.get_sites();
    auto assignments = master.assign_robots_to_crates();

    int s1 = 0, s2 = 0, s3 = 0;
    std::vector<int> all_robot_ids{};
    std::vector<std::array<double, 3> > pos_along_crate_s1{};
    std::vector<std::array<double, 3> > pos_along_crate_s2{};
    std::vector<std::array<double, 3> > pos_along_crate_s3{};
    for (const auto& assignment : *assignments) {
        if (assignment.site_id == site_ids[0]) {
            s1++;
            pos_along_crate_s1.push_back(assignment.pos_crate_frame);
        }
        else if (assignment.site_id == site_ids[1]) {
            s2++;
            pos_along_crate_s2.push_back(assignment.pos_crate_frame);
        } else if (assignment.site_id == site_ids[2]) {
            s3++;
            pos_along_crate_s3.push_back(assignment.pos_crate_frame);
        }
        all_robot_ids.push_back(assignment.robot_id);
    }
    EXPECT_EQ(s1, sites.at(site_ids[0]).robots_required);
    EXPECT_EQ(s2, sites.at(site_ids[1]).robots_required);
    EXPECT_EQ(s3, sites.at(site_ids[2]).robots_required);

    std::vector<int>::iterator it;
    const int size_all_robot_ids = all_robot_ids.size();

    it = std::unique(all_robot_ids.begin(), all_robot_ids.end());
    all_robot_ids.resize(std::distance(all_robot_ids.begin(), it));
    EXPECT_EQ(all_robot_ids.size(), size_all_robot_ids);
}

TEST(SwarmMasterTests, TestAssignmentsForTwoRobotReq) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    std::vector<int> site_ids{};
    std::vector<int> robot_ids = add_buncha_robots(&master);
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {6,4}, 1}));
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {2,8}, 1}));

    const auto& sites = master.get_sites();
    auto assignments = master.assign_robots_to_crates();
    EXPECT_EQ(assignments->size(), 4);

    std::vector<std::array<double, 3> > pos_along_crate{};
    for (const auto& assignment : *assignments)
        pos_along_crate.push_back(assignment.pos_crate_frame);

    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {-3, 0, 0}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {3, 0, 180}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {0, -4, 90}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {0, 4, 270}));
}

TEST(SwarmMasterTests, TestAssignmentsForThreeRobotReq) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    std::vector<int> site_ids{};
    std::vector<int> robot_ids = add_buncha_robots(&master);
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {6,4}, 5}));
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {2,8}, 5}));

    const auto& sites = master.get_sites();
    auto assignments = master.assign_robots_to_crates();
    EXPECT_EQ(assignments->size(), 6);

    std::vector<std::array<double, 3> > pos_along_crate{};
    for (const auto& assignment : *assignments)
        pos_along_crate.push_back(assignment.pos_crate_frame);
    
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {-3, 0, 0}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {2.8, 2, 270}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {2.8, -2, 90}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {0, -4, 90}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {1, 3.8, 180}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {-1, 3.8, 0}));
}

TEST(SwarmMasterTests, TestAssignmentsForFourRobotReq) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    std::vector<int> site_ids{};
    std::vector<int> robot_ids = add_buncha_robots(&master);
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {6,4}, 7}));

    const auto& sites = master.get_sites();
    auto assignments = master.assign_robots_to_crates();
    EXPECT_EQ(assignments->size(), 4);

    std::vector<std::array<double, 3> > pos_along_crate{};
    for (const auto& assignment : *assignments)
        pos_along_crate.push_back(assignment.pos_crate_frame);

    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {-3, 0, 0}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {3, 0, 180}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {0, -2, 90}));
    EXPECT_TRUE(found_element_in_vec(pos_along_crate, {0, 2, 270}));
}

TEST(SwarmMasterTests, TestBreakDownAssignment) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    std::vector<int> robot_ids = add_buncha_robots(&master);
    int site_id = master.add_crate_to_system({{3,2,1}, {6,5,4}, {6,4}, 5});

    std::vector<Task> true_tasks;
    typedef std::unordered_map<std::string, double> commandDict;

    true_tasks.push_back({Task::MvPlatform, commandDict{{"PlatformHeight", 0.95}}});
    true_tasks.push_back({Task::Drive, commandDict{{"ToX", 0}, {"ToY", 2}, {"ToTheta", 0}}});
    true_tasks.push_back({Task::Drive, commandDict{{"ToX", 5.8}, {"ToY", 4}, {"ToTheta", 270}}});
    true_tasks.push_back({Task::Drive, commandDict{{"ToX", 5.8}, {"ToY", 0}, {"ToTheta", 90}}});
    true_tasks.push_back({Task::MvPlatform, commandDict{{"PlatformHeight", 4}}});
    true_tasks.push_back({Task::Wait, commandDict{{"AssignmentID", site_id}}});
    true_tasks.push_back({Task::Drive, commandDict{{"ToX", 3}, {"ToY", 5}, {"ToTheta", 0}}});
    true_tasks.push_back({Task::Drive, commandDict{{"ToX", 8.8}, {"ToY", 7}, {"ToTheta", 270}}});
    true_tasks.push_back({Task::Drive, commandDict{{"ToX", 8.8}, {"ToY", 3}, {"ToTheta", 90}}});
    true_tasks.push_back({Task::Wait, commandDict{{"AssignmentID", site_id}}});
    true_tasks.push_back({Task::MvPlatform, commandDict{{"PlatformHeight", 3.95}}});

    EXPECT_FALSE(found_element_in_vec(true_tasks, Task(Task::Wait, commandDict{})));
    const auto& assignments = master.assign_robots_to_crates();
    EXPECT_EQ(assignments->size(), 3);
    for (const auto& assignment : *assignments) {
        const auto& tasks = master.break_down_assignment(assignment);
        EXPECT_EQ(tasks->size(), 7);
        for (const auto& task : *tasks) {
            EXPECT_TRUE(found_element_in_vec(true_tasks, task));
        }
    }
}

TEST(SwarmMasterTests, TestAllRobotsAtSiteWaiting) {
    SimpleClosestDesignator designator;
    SwarmMaster master(&designator);
    std::vector<int> site_ids{};
    std::vector<int> robot_ids = add_buncha_robots(&master);
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {6,4}, 1}));
    site_ids.push_back(master.add_crate_to_system({{3,2,1}, {6,5,4}, {2,8}, 5}));
    auto assignments = master.assign_robots_to_crates();

    std::vector<int> site1_robot_ids{};
    std::vector<int> site2_robot_ids{};

    for (const auto& assignment : *assignments) {
        if (assignment.site_id == site_ids[0]) site1_robot_ids.push_back(assignment.robot_id);
        else if (assignment.site_id == site_ids[1]) site2_robot_ids.push_back(assignment.robot_id);
        else EXPECT_TRUE(false);
    }

    EXPECT_FALSE(master.all_robots_at_site_waiting(site1_robot_ids[0]).first);
    EXPECT_FALSE(master.all_robots_at_site_waiting(site2_robot_ids[0]).first);
    EXPECT_TRUE(master.all_robots_at_site_waiting(site1_robot_ids[1]).first);
    EXPECT_FALSE(master.all_robots_at_site_waiting(site2_robot_ids[1]).first);
    EXPECT_TRUE(master.all_robots_at_site_waiting(site2_robot_ids[2]).first);
}

std::vector<int> add_buncha_robots(SwarmMaster* master) {
    std::vector<int> ret{};
    ret.push_back(master->add_robot_to_swarm({1,4}));
    ret.push_back(master->add_robot_to_swarm({3,2}));
    ret.push_back(master->add_robot_to_swarm({2,3}));
    ret.push_back(master->add_robot_to_swarm({5,5}));
    ret.push_back(master->add_robot_to_swarm({10,10}));
    ret.push_back(master->add_robot_to_swarm({11,11}));
    ret.push_back(master->add_robot_to_swarm({12,12}));
    ret.push_back(master->add_robot_to_swarm({13,14}));
    ret.push_back(master->add_robot_to_swarm({10,14}));
    return ret;
}

template<class Element>
bool found_element_in_vec(std::vector<Element>& pos_along_crate, Element pos) {
    return (std::find(pos_along_crate.begin(), pos_along_crate.end(), pos) != pos_along_crate.end());
}

bool operator==(const Task& t1, const Task& t2) {
    bool ret = true;
    ret = ret && (t1.task == t2.task);
    ret = ret && (t1.num_param_dict == t2.num_param_dict);
    return ret;
}