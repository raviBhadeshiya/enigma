#pragma once
/**
 * @file Enigma.hpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief      Class for Security Robot api
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
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// Cpp library
#include <cmath>
/**
 * @brief      Class for enigma for robot behavior
 */
class Enigma {
 public:
  /**
   * @brief      Constructor for Enigma.
   */
  Enigma();
  /**
   * @brief      Overloaded Constructor for Enigma
   *
   * @param[in]  n_    as ros::NodeHandle
   */
  explicit Enigma(ros::NodeHandle n_);
  /**
   * @brief      Destroys the object.
   */
  ~Enigma();
  /**
   * @brief      Laser callback for turtlebot laser sensor
   *
   * @param[in]  scan  The scan as sensor_msgs::LaserScan
   */
  void laserCallback(const sensor_msgs::LaserScan& scan);
  /**
   * @brief      Detection callback for getting the object count
   *
   * @param[in]  msg   The message as enigma::Detection
   */
  void detectionCallback(const enigma::Detection& msg);
  /**
   * @brief      Determines if any obst for sensor_msgs::LaserScan
   *
   * @param[in]  scan  The scan as sensor_msgs::LaserScan
   *
   * @return     True if any obst detected, False otherwise.
   */
  bool isObst(const sensor_msgs::LaserScan& scan);

 private:
  /**
   * @brief      mags_ as geometry_msgs::Twist
   */
  geometry_msgs::Twist msgs_;
  /**
   * @brief      laser_sub_ as ros::Subscriber
   */
  ros::Subscriber laser_sub_;
  /**
   * @brief      velocity_pub_ as ros::Publisher
   */
  ros::Publisher velocity_pub_;
  /**
   * @brief      The object counter for green and red object.
   */
  int red_ = 0, green_ = 0;
};
