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
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include "enigma/Detection.h"

class Enigma {
 public:
    Enigma();
    explicit Enigma(ros::NodeHandle n_);
    ~Enigma();
    void laserCallback(const sensor_msgs::LaserScan& scan);
    void detectionCallback(const enigma::Detection& msg);
    bool isObst(const sensor_msgs::LaserScan& scan);
 private:
    geometry_msgs::Twist msgs_;
    ros::Subscriber laser_sub_;
    ros::Publisher velocity_pub_;
    int red = 0, green = 0;
};
