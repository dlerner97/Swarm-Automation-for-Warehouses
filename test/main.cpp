/**
 * @file main.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Testing main
 * @version 0.1
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
