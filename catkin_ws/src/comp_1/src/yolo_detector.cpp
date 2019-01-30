/**
 * @file yolo_detector.cpp
 * @author Sean Scheideman
 * @date 14 Jun 2016
 * @brief Implementation for the YoloDetector class
 *
 */

#include <comp_1/yolo_detector.h>
#include <layer.h>

YoloDetector::YoloDetector(ros::NodeHandle &private_nh,
                           std::string detectorType) : it(private_nh) {
  net_ = nullptr;
  imageToProcess_.data = nullptr;
  yoloInit(private_nh);
    // image_transport::Subscriber sub;
  // image_transport::ImageTransport it(nh_);
  std::string camera;
  private_nh.getParam("cameraFront", camera);
  ROS_INFO("Camera topic: %s", camera.c_str());
  sub = it.subscribe(camera,1, &YoloDetector::imageCallback, this);
  pub = private_nh.advertise<sensor_msgs::RegionOfInterest>("roi", 1);
  pub_img = private_nh.advertise<sensor_msgs::Image>("yolo", 1);
}

YoloDetector::~YoloDetector() {
  if (net_) {
    free_network(net_);
  }

  if (imageToProcess_.data) {
    free_image(imageToProcess_);
  }
}

void YoloDetector::detect(const cv::Mat &frame) {
  // Vector to be returned
  std::vector<sensor_msgs::RegionOfInterest> roiArray;

  // Run yolo. Will fill vector: classedBoxes_
  runYolo(frame);

  // Convert YoloBoxes for ROS
  for (int i = 0; i < classedBoxes_.size(); i++) {
    std::string className = yoloClasses_[i].name_;
    float actual_width = yoloClasses_[i].actualWidth_;
    float actual_height = yoloClasses_[i].actualHeight_;

    for (auto box : classedBoxes_[i]) {
      // Calculate position and dimensions

      auto w = (unsigned int)box.w;
      auto h = (unsigned int)box.h;

      sensor_msgs::RegionOfInterest roi;
      roi.x_offset = box.x;
      roi.y_offset = box.y;
      roi.width = w;
      roi.height = h;
      // roi.actualWidth = actual_width;
      // roi.actualHeight = actual_height;

      /*
      double distance = au_core::calculateDistance(w, h, actual_width,
                                                   actual_height, cameraInfo);
      auto x_error = (int)(roi.topLeft.x + (roi.width / 2) - cameraInfo.cx);
      auto y_error = (int)(roi.topLeft.y + (roi.height / 2) - cameraInfo.cy);

      
      // In degrees
      roi.poseEstimate.position.x =
          au_core::calculateAngle(x_error, cameraInfo.width, cameraInfo.fov);
      // In metres
      roi.poseEstimate.position.y =
          au_core::calculateLateral(y_error, cameraInfo.fy, (float)distance) /
          100.0;
      // In metres
      roi.poseEstimate.position.z = distance / 100.0;
      */
      // First tag is by default the one drawn
      // roi.tags.push_back(className);

      // Register addition
      roiArray.push_back(roi);
    }
  }
  cv::Mat clone = frame.clone();
  if (roiArray.size()) {
    pub.publish(roiArray[0]);
    debugDraw(clone, roiArray[0]);
  }

  sensor_msgs::Image img_msg; // >> message to be sent

  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  img_bridge_ = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, clone);
  img_bridge_.toImageMsg(img_msg);
  pub_img.publish(img_msg);
}

void YoloDetector::debugDraw(cv::Mat &frame, sensor_msgs::RegionOfInterest& roi) {
  putText(frame, "robot", cv::Point(15, 15),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 255), 1,
          CV_AA);
    cv::Rect rect(roi.x_offset, roi.y_offset, roi.width, roi.height);
    cv::rectangle(frame, rect, cv::Scalar(255, 0, 0), 2,
                  CV_AA);
}

void YoloDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::Mat temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    detect(temp);
    // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    // cv::waitKey(30);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void YoloDetector::yoloInit(ros::NodeHandle nodeHandle) {
  // Status update
  ROS_INFO("[yolo_detector] yoloInit().");

  // Initialize deep network of darknet.
  std::string cfgModel, weightsModel;
  std::vector<std::string> classNames;
  std::vector<float> classWidths, classHeights;
  char *cfg;
  char *weights;
  char **names;

  nodeHandle.getParam("class_names", classNames);
  nodeHandle.getParam("class_widths", classWidths);
  nodeHandle.getParam("class_heights", classHeights);
  // make sure each class has a name, height and width
  if ((classNames.size() != 0) && (classNames.size() == classWidths.size()) &&
      (classNames.size() == classHeights.size())) {
    numYoloClasses_ = classNames.size();
    classedBoxes_.resize(numYoloClasses_);
    names = new char *[numYoloClasses_];

    for (int i = 0; i < numYoloClasses_; ++i) {
      yoloClasses_.push_back(
          YoloClass(classNames[i], classWidths[i], classHeights[i]));
    }
  } else {
    ROS_INFO("Each class must have a name, width and height");
    ros::shutdown();
  }

  // Threshold of object detection.
  float thresh;
  nodeHandle.param("object_threshold", thresh, 0.3f);

  // Path to weights file.
  nodeHandle.param("weights_model", weightsModel,
                   std::string("yolo-arvp.weights"));
  ROS_INFO("%s", weightsModel.c_str());
  std::string weightsPath =
      ros::package::getPath("au_vision") + "/weights/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle.param("cfg_model", cfgModel, std::string("yolo-arvp.cfg"));
  std::string configPath =
      ros::package::getPath("au_vision") + "/cfg/" + cfgModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Get classes.
  for (int i = 0; i < numYoloClasses_; i++) {
    //+1 is for null char
    char *name = new char[yoloClasses_[i].name_.length() + 1];
    strcpy(name, yoloClasses_[i].name_.c_str());
    names[i] = name;
  }

  // Load network.
  loadNetwork(cfg, weights, thresh, names, numYoloClasses_, 0.5f, 0.4f);

  // Free allocated memory
  for (int i = 0; i < numYoloClasses_; ++i) {
    delete names[i];
  }
  delete[] names;

  delete weights;
  delete cfg;
}

void YoloDetector::loadNetwork(char *cfgfile, char *weightfile, float thresh,
                               char **names, int classes, float hier,
                               float nms) {
  thresh_ = thresh;
  hier_ = hier;
  nms_ = nms;

  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
  network_size_ = cv::Size(net_->w, net_->h);

  srand(2222222);
}

image YoloDetector::matToImage(const cv::Mat &src) {
  // Special case: return empty
  if (src.empty()) {
    image out = make_empty_image(0, 0, 0);
    return out;
  }

  // originally this should be a letterbox image. However, simple resize to fit
  // the network didn't seem to affect accuracy.
  cv::Mat resized_image;
  cv::resize(src, resized_image, network_size_);
  // Convert cv::Mat to image
  // cv::Mat is in uchar whereas image is in floats
  // src.data is a 1D array where each element alterates between the color
  // channels ie. index 0, 1, 2 is BGR of the pixel
  image out = make_image(resized_image.cols, resized_image.rows,
                         resized_image.channels());
  // cv::split seperates src into its three color channels
  std::vector<cv::Mat> channels;
  cv::split(resized_image, channels);
  // out.data is a 1D array of size height*width*channels where each third of
  // the array contains values of the same channel
  long int image_size = resized_image.cols * resized_image.rows;
  for (int i = 0; i < channels.size(); ++i) {
    long int image_offset = image_size * i;
    channels[i].convertTo(channels[i], CV_32FC1, 1.0 / 255);
    memcpy(out.data + image_offset, channels[i].data,
           image_size * sizeof(float));
  }

  return out;
}

void YoloDetector::runYolo(const cv::Mat &fullFrame) {
  // Ensure network was loaded
  ROS_ASSERT(net_);

  // Convert image to darknet format
  if (imageToProcess_.data) {
    free_image(imageToProcess_);
  }
  imageToProcess_ = matToImage(fullFrame);

  network_predict(net_, imageToProcess_.data);

  int detectionCount = 0;
  detection *detectionResults = get_network_boxes(
      net_, net_->w, net_->h, thresh_, hier_, nullptr, 1, &detectionCount);

  // Remove detections which overlap too much
  layer l = net_->layers[net_->n - 1];
  if (nms_ > 0) {
    do_nms_sort(detectionResults, detectionCount, l.classes, nms_);
  }

  // Convert results to YoloBox objects
  std::vector<YoloBox> candidates;
  for (int i = 0; i < detectionCount; ++i) {
    YoloBox c_box;

    // Classify each detection
    c_box.whichClass = getYoloClass(detectionResults[i], numYoloClasses_);
    c_box.prob = detectionResults[i].prob[c_box.whichClass];

    if (c_box.prob) {
      box b = detectionResults[i].bbox;

      // taken exactly from image.c draw_detections
      int left = (b.x - b.w / 2.) * fullFrame.cols;
      int right = (b.x + b.w / 2.) * fullFrame.cols;
      int top = (b.y - b.h / 2.) * fullFrame.rows;
      int bot = (b.y + b.h / 2.) * fullFrame.rows;

      if (left < 0) left = 0;
      if (right > fullFrame.cols - 1) right = fullFrame.cols - 1;
      if (top < 0) top = 0;
      if (bot > fullFrame.rows - 1) bot = fullFrame.rows - 1;

      c_box.x = left;
      c_box.y = top;
      c_box.w = right - left;
      c_box.h = bot - top;

      // Add to vector
      candidates.push_back(c_box);
    }
  }

  // Free detections and images
  free_detections(detectionResults, detectionCount);
  free_image(imageToProcess_);
  imageToProcess_.data = nullptr;

  // Clear categories
  for (int i = 0; i < numYoloClasses_; i++) {
    classedBoxes_[i].clear();
  }

  // categorize bounding boxes by class
  if (!candidates.empty()) {
    // display number of objects
    // ROS_INFO("# Objects: %lu", candidates.size());
    for (YoloBox &candidate : candidates) {
      for (int j = 0; j < numYoloClasses_; j++) {
        if (candidate.whichClass == j) {
          classedBoxes_[j].push_back(candidate);

          // display confidence of each detection
          // ROS_INFO("%s (%f%%)", yoloClasses_[j].name_.c_str(),
          //          candidate.prob * 100);
        }
      }
    }
  }
}

int YoloDetector::getYoloClass(const detection &d, int num_classes) const {
  // The index with the maximum probability is the class of the
  // detection

  float max_prob = 0;
  int max_index = 0;

  for (int c = 0; c < num_classes; c++) {
    if (d.prob[c] > max_prob) {
      max_prob = d.prob[c];
      max_index = c;
    }
  }

  return max_index;
}
