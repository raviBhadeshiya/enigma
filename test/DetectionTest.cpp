/**
 * @file DetectionTest.cpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief This file contain all the test cases for Detection
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
#include <ros/service_client.h>
// Custom library
#include "enigma/Detection.h"
#include "enigma/Detection.hpp"
#include "enigma/startStop.h"

/**
 * @brief      Struct for service callback helper
 */
struct TestHelper {
  /**
   * @brief      Construct the object
   */
  TestHelper() : count(0), red(0), green(0) {}
  /**
   * @brief      Simple callBack function
   *
   * @param[in]  msg   The message as enigma::Detection
   */
  void cb(const enigma::Detection& msg) {
    green = msg.green;
    red = msg.red;
    ++count;
  }
  uint32_t count;
  uint32_t red;
  uint32_t green;
};
/**
 * @brief      To test for Detection
 */
TEST(TEST_DETECTION, TestOpenCvInit) {
  ros::NodeHandle n_;
  EXPECT_NO_FATAL_FAILURE(Detection test(n_));
  EXPECT_TRUE(1);
}
/**
 * @brief      To test the image subscriber is working properly
 */
TEST(TEST_DETECTION, TestSubscriber) {
  ros::NodeHandle n_;
  // Should Subscribe by over detection_node
  auto testingPub =
      n_.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 0);
  ros::WallDuration(0.4).sleep();
  EXPECT_EQ(testingPub.getNumSubscribers(), 1);
}
/**
 * @brief      To test the Detection publisher
 */
TEST(TEST_DETECTION, TestPublisher) {
  ros::NodeHandle n_;
  TestHelper h;
  // Detection publisher should subscribed by this
  auto test_sub = n_.subscribe("detection", 1, &TestHelper::cb, &h);

  ros::WallDuration(2).sleep();
  ros::spinOnce();

  EXPECT_EQ(test_sub.getNumPublishers(), 1);
  EXPECT_NE(h.count, 0);
  EXPECT_EQ(h.red, 2);
  EXPECT_EQ(h.green, 1);
}
/**
 * @brief      To test the detection method
 */
TEST(TEST_DETECTION, TestDetection) {
  ros::NodeHandle n_;
  Detection testDetect(n_);
  // Subscribe the image topic to get it from gazebo world
  auto test_sub = n_.subscribe<sensor_msgs::Image>(
      "/camera/rgb/image_raw", 1, &Detection::imageCallBack, &testDetect);

  ros::WallDuration(1).sleep();
  ros::spinOnce();

  cv::Mat testImage;
  int red = 2, green = 1;
  // Check detection is working or not.
  EXPECT_NO_FATAL_FAILURE(testImage = testDetect.getImage());
  EXPECT_EQ(std::get<0>(testDetect.detect(testImage)), red);
  EXPECT_EQ(std::get<1>(testDetect.detect(testImage)), green);
}

/**
 * @brief      Test detection start and stop service
 */
TEST(TEST_DETECTION, TestSwitchService) {
  ros::NodeHandle n_;
  Detection testDetect(n_);

  auto client = n_.serviceClient<enigma::startStop>("detectionSwitch");

  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);

  // Check service and response
  enigma::startStop srv;
  srv.request.query = false;
  EXPECT_TRUE(testDetect.switchServiceCB(srv.request, srv.response));
  EXPECT_FALSE(srv.response.state);

  srv.request.query = true;
  EXPECT_TRUE(testDetect.switchServiceCB(srv.request, srv.response));
  EXPECT_TRUE(srv.response.state);
}

