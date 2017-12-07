#pragma once
/**
 * @file Detection.hpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief      Threat Detection Class
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
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <enigma/Detection.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <utility>
#include <vector>
#include <iostream>

class Detection {
 public:
  Detection();
  explicit Detection(ros::NodeHandle n);
  ~Detection();
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
  std::pair<int, int> detect(const cv::Mat& image);
  cv::Mat postProcessing(const cv::Mat& image);
  int countBlob(const cv::Mat& image);
  cv::Mat getImage();
 private:
  cv_bridge::CvImagePtr cv_ptr;
  ros::Subscriber image_sub_;
  ros::Publisher detection_pub_;
  enigma::Detection message;
};
