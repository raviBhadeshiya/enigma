/**
 * @file Detection.cpp
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

#include "enigma/Detection.hpp"

Detection::Detection() {}

Detection::Detection(ros::NodeHandle n) {
  ROS_INFO("Detection init successfully..");

  image_sub_ = n.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 10,
                                               &Detection::imageCallBack, this);

  detection_pub_ = n.advertise<enigma::Detection>("detection", 10);
}

Detection::~Detection() {}

void Detection::imageCallBack(const sensor_msgs::ImageConstPtr &msg) {
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  int red = 0, green = 0;
  // auto [red, green] = detect(cv_ptr->image);
  std::tie(red, green) = detect(cv_ptr->image);

  if (red != 0 || green != 0) {
    message.red = red;
    message.green = green;
    detection_pub_.publish(message);
  }
}

std::pair<int, int> Detection::detect(const cv::Mat &image) {
  cv::Mat hsv_image;

  cv::cvtColor(image, hsv_image, CV_BGR2HSV);

  cv::Mat redImgThresholded, greenImgThresholded;

  cv::inRange(hsv_image, cv::Scalar(0, 110, 0), cv::Scalar(50, 255, 255),
              redImgThresholded);  // Threshold the image
  redImgThresholded = postProcessing(redImgThresholded);

  cv::inRange(hsv_image, cv::Scalar(20, 110, 0), cv::Scalar(179, 255, 255),
              greenImgThresholded);  // Threshold the image

  greenImgThresholded = postProcessing(greenImgThresholded);

  return std::make_pair(countBlob(redImgThresholded),
                        countBlob(greenImgThresholded));
}

cv::Mat Detection::postProcessing(const cv::Mat &image) {
  // morphological opening (remove small objects from the foreground)

  auto processedImage = image;

  cv::erode(processedImage, processedImage,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(processedImage, processedImage,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  // morphological closing (fill small holes in the foreground)
  cv::dilate(processedImage, processedImage,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::erode(processedImage, processedImage,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  return processedImage;
}

int Detection::countBlob(const cv::Mat &image) {
  // Vector for storing contour
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image, contours, hierarchy, CV_RETR_CCOMP,
                   CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image
  return contours.size();
}

cv::Mat Detection::getImage() {
  return cv_ptr->image;
}
