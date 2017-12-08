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

Enigma::Enigma() {}

Enigma::Enigma(ros::NodeHandle n_) {
  velocity_pub_ =
      n_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);

  laser_sub_ = n_.subscribe("/scan", 10, &Enigma::laserCallback, this);
  // Reset the the velocity
  msgs_.linear.x = 0;
  msgs_.angular.z = 0;
  velocity_pub_.publish(msgs_);

  ROS_INFO("Enigma init successfully..");
}

Enigma::~Enigma() {
  ROS_INFO("Turtlebot walker is shutting down..");
  // On Destruction stop the robot
  msgs_.linear.x = 0;
  msgs_.angular.z = 0;
  velocity_pub_.publish(msgs_);
}

void Enigma::laserCallback(const sensor_msgs::LaserScan& scan) {
  ROS_DEBUG("LaserCallback called!");
  // If obst is nearby go to turning behavior

  if (isObst(scan)) {
    msgs_.linear.x = 0;
    msgs_.angular.z = (red != 0) ? 0.7 : -0.7;
    velocity_pub_.publish(msgs_);
    ROS_WARN("Obst detected!--> Turnning..");
  } else {
    // Else walk straight
    msgs_.linear.x = 0.8;
    msgs_.angular.z = 0;
    velocity_pub_.publish(msgs_);
    ROS_DEBUG("Straight..");
  }
}

void Enigma::detectionCallback(const enigma::Detection& msg) {
  green = msg.green;
  red = msg.red;
}

bool Enigma::isObst(const sensor_msgs::LaserScan& scan){
  size_t angle_thresold = 25;
  double dist_thresold = 1.5;

  size_t range =
      std::round(angle_thresold * (M_PI / 180) / scan.angle_increment);
  size_t size = scan.ranges.size() / 2;
  size_t count = 0;

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
