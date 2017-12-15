#pragma once
/**
 * @file Detection.hpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief      Anomaly detection class
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
#include <enigma/Detection.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
// OpenCv library
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Custom service library
#include <enigma/startStop.h>
// Cpp library
#include <iostream>
#include <utility>
#include <vector>
/**
 * @brief      Class for detection.
 */
class Detection {
 public:
  /**
   * @brief      Constructor for Detection.
   */
  Detection();
  /**
   * @brief      Overloaded Constructor for Detection.
   *
   * @param[in]  n     as ros::NodeHandle
   */
  explicit Detection(ros::NodeHandle n);
  /**
   * @brief      Destroys the Detecion.
   */
  ~Detection();
  /**
   * @brief      Image callBack for turtlebot camera
   *
   * @param[in]  msg   The message as sensor_mags::ImageConstPtr
   */
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
  /**
   * @brief      The simple function for detecting the red and green objects
   *
   * @param[in]  image  The image as cv::Mat
   *
   * @return     objects count as pair<red=int, green=int>>
   */
  std::pair<int, int> detect(const cv::Mat& image);
  /**
   * @brief      Gets the image.
   *
   * @return     The image as cv::Mat.
   */
  cv::Mat getImage();
  /**
   * @brief      Start stop service callback
   *
   * @param      req   The request as bool
   * @param      res   The resource as bool
   *
   * @return     True if worked
   */
  bool switchServiceCB(enigma::startStop::Request &req,
                       enigma::startStop::Response &res);

 private:
  /**
   * @brief     cv_ptr as cv_bridge::CvImagePtr for converting ros image.
   */
  cv_bridge::CvImagePtr cv_ptr;
  /**
   * @brief      The image subscriber for /camera/rgb/image_raw topic.
   */
  ros::Subscriber image_sub_;
  /**
   * @brief      The detection publisher for /detection
   */
  ros::Publisher detection_pub_;
  /**
   * @brief      The start stop service server
   */
  ros::ServiceServer switchServer_;
  /**
   * @brief      The custom message for detection publisher
   */
  enigma::Detection message;
  /**
   * @brief      postProcessing the image for clearing the noise
   *
   * @param[in]  image  The image as cv::Mat
   *
   * @return     processed image as cv::Mat
   */
  cv::Mat postProcessing(const cv::Mat& image);
  /**
   * @brief      Counts the number of blob.
   *
   * @param[in]  image  The image as cv::Mat
   *
   * @return     Number of blob as int.
   */
  int countBlob(const cv::Mat& image);
  /**
   * @brief      The bool switch start and stopping the detection
   */
  bool switch_ = true;
};
