/**
 * @file detector_handler.cpp
 * @author Jacky Chung
 * @date 21 Mar 2018
 * @brief service for continuous object detection
 *
 * Instantiates the detector object and starts server
 * A string message containing the name of a detector
 * can be sent to /detector_type topic to change the
 * detector
 *
 * Essential ros params:
 *  * detector: type of detector (string)
 *
 */
#include <ros/ros.h>
#include <string>

// #include <au_core/sigint_handler.h>

// #include <au_vision/detector_handler/detector_handler.h>
#include <comp_1/yolo_detector.h>

int main(int argc, char** argv) {
//   au_core::handleInterrupts(argc, argv, "detector_handler", true);
  ros::init(argc, argv, "yolo_detector");

  ros::NodeHandle private_nh("~");

  YoloDetector yoloDetector(private_nh, "yolo_detector");
//   DetectorHandler detectorHandler;  // initialize detectorHandler object to
                                    // dynamically switch detectors
  // get rate param
  int spinRate = 5;

  ros::Rate rate(spinRate);
  while (true) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
