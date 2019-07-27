/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Pointcloud Registration C++/ROS Kinect 360-featured base%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                 %
The MIT License

Copyright (c) 2010-2019 Google, Inc. 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <time.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"


#include <pcl_ros/point_cloud.h> //pcl
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h> // message_filters
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>  // OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/nonfree.hpp>

#include <eigen3/Eigen/Dense>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "map_recon.h"

//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

namespace map_recon {

class MapReconNodeHandler {
 public:

  MapRecon MapManager;
  bool FirtIteration = true;
  
  // class Constructor
  MapReconNodeHandler(ros::NodeHandle& node_handler);
  
  // class Destructor
  virtual ~MapReconNodeHandler();

  void rgb_img_callback(const sensor_msgs::Image::ConstPtr& input_img);
  void depth_img_callback(const sensor_msgs::Image::ConstPtr& input_img);
  void kinect_callback(const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::ImageConstPtr& msg_rgb);

  void tf_publisher(string frame_id, string child_frame_id, ros::Time time_stamp, Eigen::Matrix4f transformation);

  void TransformTFToEigenImpl(const tf::Transform &t, Eigen::Affine3d &e);
 private:
  bool ReadConfig_map();
  bool ReadConfig_run(MapRecon& Map_manager);

  // ROS node handle.
  ros::NodeHandle nodeHandle_;

  message_filters::Subscriber<sensor_msgs::Image> subscriber_depth;
  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb;

  #ifdef EXACT
    typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  #endif
  #ifdef APPROXIMATE
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  #endif
  typedef Synchronizer<MySyncPolicy> sync;
  boost::shared_ptr<sync> sync_;

  ros::Publisher pcl_Publisher_;
  ros::Publisher map_Publisher_;
  ros::Publisher depth_image_Publisher_;

  std::string rgb_image_SubscriberTopic;
  std::string depth_image_SubscriberTopic;
  std::string tf_SubscriberTopic;

  std::string pcl_PublisherTopic;
  std::string map_PublisherTopic;
  std::string depthImage_PublisherTopic;

  tf::TransformListener kinect_listener;
  tf::TransformBroadcaster kinect_broadcaster;
  tf::StampedTransform kinect_transform;

};
}  // namespace map_recon ...

