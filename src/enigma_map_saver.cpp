/**
 * @file enigma_map_saver.cpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief      Map saver service
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

#include <unistd.h>
#include <string>
#include "enigma/fileSave.h"
#include "ros/ros.h"

bool serviceCallBack(enigma::fileSave::Request &req,
                     enigma::fileSave::Response &res) {
  char *name[4];
  res.check = false;

  name[0] = "/bin/bash";
  name[1] = "-c";
  // rosrun octomap_saver octomap_saver -f /tmp/temp.ot
  // std::string file = "rosrun octomap_saver octomap_saver -f /tmp/" +
  name[2] = "rosrun octomap_server octomap_saver -f $(rospack find enigma)/map/enigma_map.ot";
  // name[2] = "roslaunch enigma enigma_map_saver.launch";
  name[3] = NULL;
  res.check = true;
  execvp(name[0], name);

  ROS_INFO("Map saved\n");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "enigm_map_saver");

  ros::NodeHandle n;
  auto service = n.advertiseService("save_map", serviceCallBack);

  ROS_INFO("Ready to save files");
  ros::spin();

  return 0;
}
