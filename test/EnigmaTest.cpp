/**
 * @file EnigmaTest.cpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief This file contain all the test cases for Enigma
 *
 * @copyright  MIT License (c) 2017 Ravi Bhadeshiya
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// ROS library
#include <gtest/gtest.h>
#include <ros/ros.h>
// Cpp library
#include <memory>
// Custom library
#include "enigma/Enigma.hpp"
#include "enigma/changeSpeed.h"
#include "enigma/startStop.h"

/**
 * @brief      To test Enigma
 */
TEST(TEST_Enigma, TestInit) {
  ros::NodeHandle n_;
  EXPECT_NO_FATAL_FAILURE(Enigma test(n_));
}
/**
 * @brief      Test behavior without obst
 */
TEST(TEST_Enigma, noObst) {
  // laser scan to provide a fake non-obst:
  size_t num_readings = 50;

  sensor_msgs::LaserScan scan;
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1 / 40) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 100.0;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for (auto& i : scan.ranges) {
    i = scan.range_max;
  }

  ros::NodeHandle n_;
  Enigma robot(n_);
  // Should not detect the obst
  EXPECT_FALSE(robot.isObst(scan));
  EXPECT_NO_FATAL_FAILURE(robot.laserCallback(scan));
}
/**
 * @brief      Test behavior with obst
 */
TEST(TEST_Enigma, obst) {
  // laser scan to provide a fake obst:
  size_t num_readings = 50;

  sensor_msgs::LaserScan scan;
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1 / 40) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 100.0;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for (auto& i : scan.ranges) {
    i = scan.range_min;
  }

  ros::NodeHandle n_;
  Enigma robot(n_);
  // should detect the obst
  EXPECT_TRUE(robot.isObst(scan));
  EXPECT_NO_FATAL_FAILURE(robot.laserCallback(scan));
}
/**
 * @brief      Test detection callback
 */
TEST(TEST_Enigma, detecionCB) {
  ros::NodeHandle n_;
  Enigma robot(n_);
  enigma::Detection msg;
  msg.red = 1;
  msg.green = 2;
  // Able to process
  EXPECT_NO_FATAL_FAILURE(robot.detectionCallback(msg));
}
/**
 * @brief      Test robot start and stop service
 */
TEST(TEST_Enigma, TestSwitchService) {
  ros::NodeHandle n_;
  Enigma robot(n_);

  auto client = n_.serviceClient<enigma::startStop>("robotSwitch");

  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);

  // Check service and response
  enigma::startStop srv;
  srv.request.query = false;
  EXPECT_TRUE(robot.switchServiceCB(srv.request, srv.response));
  EXPECT_FALSE(srv.response.state);

  srv.request.query = true;
  EXPECT_TRUE(robot.switchServiceCB(srv.request, srv.response));
  EXPECT_TRUE(srv.response.state);
}
/**
 * @brief      Test robot speed change service
 */

TEST(TEST_Enigma, TestSpeedService) {
  ros::NodeHandle n_;
  Enigma robot(n_);

  auto client = n_.serviceClient<enigma::startStop>("robotSpeed");

  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);

  // Check service and response
  enigma::changeSpeed srv;
  // invalid request more than 1.0
  srv.request.speed = 1.1;
  EXPECT_FALSE(robot.speedServiceCB(srv.request, srv.response));
  EXPECT_FALSE(srv.response.success);
  // valid request
  srv.request.speed = 0.8;
  EXPECT_TRUE(robot.speedServiceCB(srv.request, srv.response));
  EXPECT_TRUE(srv.response.success);
}
