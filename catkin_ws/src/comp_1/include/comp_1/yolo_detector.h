#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/RegionOfInterest.h"

#include <opencv2/opencv.hpp>

#ifdef GPU
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "curand.h"
#endif

extern "C" {
#include "box.h"
#include "cost_layer.h"
#include "detection_layer.h"
#include "image.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
}

#ifndef COMP_1_YOLO_DETECTOR_H
#define COMP_1_YOLO_DETECTOR_H

// YoloDetector

class YoloDetector {
 public:
  YoloDetector(ros::NodeHandle& private_nh,
               std::string detectorType = "yolo_detector");
  ~YoloDetector();

 protected:
//   std::vector<au_core::Roi> detect(const cv::Mat& frame,
//                                    const au_core::CameraInfo& cameraInfo);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void detect(const cv::Mat& frame);
 private:
  // Type for bounding boxes of detected objects.
  struct YoloBox {
    int x, y, w, h;
    float prob;
    int whichClass;
  };

  // Contains information for each type of detected object
  struct YoloClass {
    YoloClass(std::string name, float width = 0, float height = 0)
        : name_(name), actualWidth_(width), actualHeight_(height){};
    std::string name_;
    float actualWidth_;
    float actualHeight_;
  };

  int numYoloClasses_;
  std::vector<YoloClass> yoloClasses_;
  std::vector<std::vector<YoloBox> > classedBoxes_;  // Divided into classes

  network* net_;
  cv::Size network_size_;
  image imageToProcess_;
  float hier_;
  float thresh_;
  float nms_;

  image_transport::Subscriber sub;
  image_transport::ImageTransport it;
  ros::Publisher pub;
  ros::Publisher pub_img;
  cv_bridge::CvImage img_bridge_;
  /**
   * @brief Initialize darknet network of yolo.
   * @param[in] cfgfile location of darknet's cfg file describing the layers of
   * the network.
   * @param[in] weightfile location of darknet's weights file setting the
   * weights of the network.
   * @param[in] thresh threshold of the object detection (0 < thresh < 1).
   */
  void loadNetwork(char* cfgfile, char* weightfile, float thresh, char** names,
                   int classes, float hier, float nms);

  image matToImage(const cv::Mat& src);

  void debugDraw(cv::Mat &frame, sensor_msgs::RegionOfInterest& roi);

  /**
   * @brief yoloInit, Loads trained weights and initialize the yolo network.
   * @param nodeHandle, node handle for ros process
   */
  void yoloInit(ros::NodeHandle nodeHandle);

  /**
   * @brief runYolo, runs YOLO inference on input frame
   */
  void runYolo(const cv::Mat& fullFrame);

  /**
   * @breif Returns the most likely class of a detection candidate
   */
  int getYoloClass(const detection& d, int num_classes) const;
};

#endif  // COMP_1_YOLO_DETECTOR_H
