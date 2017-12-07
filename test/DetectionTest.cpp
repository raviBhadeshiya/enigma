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
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "enigma/Detection.h"
#include "enigma/Detection.hpp"

struct TestHelper {
  TestHelper() : count(0), red(0), green(0) {}
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
 * @brief      To test Detection
 */
TEST(TEST_DETECTION, TestOpenCvInit) {
  ros::NodeHandle n_;
  EXPECT_NO_FATAL_FAILURE(Detection test(n_));
  EXPECT_TRUE(1);
}

TEST(TEST_DETECTION, TestSubscriber) {
  ros::NodeHandle n_;
  // Should Subscribe by over detection_node
  auto testingPub =
      n_.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 0);
  ros::WallDuration(0.4).sleep();
  EXPECT_EQ(testingPub.getNumSubscribers(), 1);
}

TEST(TEST_DETECTION, TestPublisher) {
  ros::NodeHandle n_;
  TestHelper h;
  // Detection publisher should subscribe by this
  auto test_sub = n_.subscribe("detection", 1, &TestHelper::cb, &h);

  ros::WallDuration(5).sleep();
  ros::spinOnce();

  EXPECT_EQ(test_sub.getNumPublishers(), 1);
  EXPECT_NE(h.count, 0);
  EXPECT_EQ(h.red, 2);
  EXPECT_EQ(h.green, 1);
}

TEST(TEST_DETECTION, TestDetection) {
  ros::NodeHandle n_;
  Detection testDetect(n_);

  auto test_sub = n_.subscribe<sensor_msgs::Image>(
      "/camera/rgb/image_raw", 1, &Detection::imageCallBack, &testDetect);

  ros::WallDuration(5).sleep();
  ros::spinOnce();

  cv::Mat testImage;
  int red = 2, green = 1;

  EXPECT_NO_FATAL_FAILURE(testImage = testDetect.getImage());
  EXPECT_EQ(std::get<0>(testDetect.detect(testImage)), red);
  EXPECT_EQ(std::get<1>(testDetect.detect(testImage)), green);
}
