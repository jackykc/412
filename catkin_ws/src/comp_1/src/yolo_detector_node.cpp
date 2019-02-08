/**
 * Starts the yolo detector
 */
#include <ros/ros.h>
#include <string>

#include <comp_1/yolo_detector.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolo_detector");

  ros::NodeHandle private_nh("~");

  YoloDetector yoloDetector(private_nh, "yolo_detector");
  int spinRate = 10;

  ros::Rate rate(spinRate);
  while (true) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
