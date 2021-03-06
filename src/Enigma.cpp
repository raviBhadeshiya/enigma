/**
 * @file Enigma.cpp
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
#include "enigma/Enigma.hpp"
// Constructor for Enigma
Enigma::Enigma() {}
// Overloaded Constructor for Enigma
Enigma::Enigma(ros::NodeHandle n_) {
  // Velocity publisher for turtlebot
  velocity_pub_ =
      n_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
  // Laser message subscribe
  laser_sub_ = n_.subscribe("/scan", 10, &Enigma::laserCallback, this);

  // Service
  switchServer_ =
      n_.advertiseService("robotSwitch", &Enigma::switchServiceCB, this);
  speedServer_ =
      n_.advertiseService("robotSpeed", &Enigma::speedServiceCB, this);
  // Reset the the velocity
  msgs_.linear.x = 0;
  msgs_.angular.z = 0;
  // Publisher the stop msg_ first
  velocity_pub_.publish(msgs_);

  ROS_INFO("Enigma init successfully..");
}
// Destroys the object
Enigma::~Enigma() {
  ROS_INFO("Turtlebot walker is shutting down..");
  // On Destruction stop the robot
  msgs_.linear.x = 0;
  msgs_.angular.z = 0;
  velocity_pub_.publish(msgs_);
}
// laserCallback for exploratory behavior
void Enigma::laserCallback(const sensor_msgs::LaserScan& scan) {
  ROS_DEBUG("LaserCallback called!");

  // Switch toggle for robot
  float speed;
  if (switch_)
    speed = maxSpeed_;
  else
    speed = 0;

  // If obst is nearby go to turning behavior
  // If any obst detected than turn
  if (isObst(scan)) {
    msgs_.linear.x = 0;
    msgs_.angular.z = speed - 0.1;
    velocity_pub_.publish(msgs_);
    ROS_DEBUG("Obst detected!--> Turnning..");
  } else {
    // Else walk straight
    msgs_.linear.x = speed;
    msgs_.angular.z = 0;
    velocity_pub_.publish(msgs_);
    ROS_DEBUG("Straight..");
  }
}
// Detection callback for getting data
void Enigma::detectionCallback(const enigma::Detection& msg) {
  green_ = msg.green;
  red_ = msg.red;
  if (red_ != 0 || green_ != 0) {
    ROS_INFO(
      "Anomaly detected!!Red Object found:%d and Green Object found:%d",
             red_, green_);
  }
}
// To check is there any obst for collision
bool Enigma::isObst(const sensor_msgs::LaserScan& scan) {
  // angle_thresold
  size_t angle_thresold = 25;
  // Dist thresold
  double dist_thresold = 1.5;
  // Find the range based on thresold
  size_t range =
      std::round(angle_thresold * (M_PI / 180) / scan.angle_increment);
  size_t size = scan.ranges.size() / 2;
  size_t count = 0;
  // Logic for any obst
  for (const auto& itr : scan.ranges) {
    // scan->range_min is 0.45
    if (!(count < (size - range) || count > (size + range))) {
      if (itr <= scan.range_min + dist_thresold) {
        return true;
      }
    }
    count++;
  }
  return false;
}
// speed change service call back
bool Enigma::speedServiceCB(enigma::changeSpeed::Request& req,
                            enigma::changeSpeed::Response& res) {
  if (0.0 <= req.speed && req.speed <= 1.0) {
    res.success = true;
    maxSpeed_ = req.speed;
    ROS_INFO("Robot speed change to %f", maxSpeed_);
  } else {
    res.success = false;
    ROS_INFO("Invalid speed!");
    return false;
  }

  return true;
}
// to start and stop the robot callback
bool Enigma::switchServiceCB(enigma::startStop::Request& req,
                             enigma::startStop::Response& res) {
  switch_ = req.query;

  if (switch_) {
    ROS_INFO("Robot is starting!");
    res.state = true;
  } else {
    ROS_INFO("Robot is stopping!");
    res.state = false;
  }
  return true;
}
