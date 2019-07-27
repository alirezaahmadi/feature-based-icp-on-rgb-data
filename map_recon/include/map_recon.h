/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Pointcloud Registration C++/ROS Kinect 360-featured base%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AhmadiAlireza.webs.com                                 %
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
#include <opencv2/xfeatures2d.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/flann/miniflann.hpp"

#include <stdlib.h>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <time.h>   

#include <algorithm>


#include <eigen3/Eigen/Dense>

using namespace std;
using namespace message_filters;
using namespace pcl;
using namespace cv;
using namespace Eigen;
using namespace cv::xfeatures2d;

using cv::xfeatures2d::SiftFeatureDetector;
using cv::xfeatures2d::SiftDescriptorExtractor;


namespace map_recon {

struct Feature2D {
  string name;
  uint frameNum;
  uint FeatureNum;
  float pose_x;
  float pose_y;
  float ORB[500];
  float SIFT[128];
  float SURF[64];
};

class MapRecon {
 public:

  // calss Constructor
  MapRecon();
  // calss deconstructor

  virtual ~MapRecon();

  //void draw_cloud(const std::string &text, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  vector<vector<int>> CorPointsPaternBased(uint8_t Method, uint SampleSize, uint pcl_size);

  double get_distance(PointXYZRGB& P_new, PointXYZRGB& P_old);

  uint get_nearest_neighbour(PointXYZRGB& P_new, PointCloud<PointXYZRGB>::Ptr& pcl_old, double& FminDistance);

  void update_CoPoints_NN(PointCloud<PointXYZRGB>::Ptr& pcl_new,PointCloud<PointXYZRGB>::Ptr& pcl_old);

  void get_CorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pcl, const cv::Mat& image_rgb, const cv::Mat& image_depth);

  void filter_point_cloud(PointCloud<PointXYZRGB>::Ptr& pc, PointCloud<PointXYZRGB>::Ptr& pc_f, float minZ, float maxZ);

  void load_pcl_file(string address,PointCloud<PointXYZRGB>::Ptr& pcf);

  // function: Transformation DRGB --> XYZRGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(const cv::Mat& image_rgb, const cv::Mat &image_depth);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_to_cloud(const cv::Mat& image_rgb, const cv::Mat &image_depth, vector<cv::KeyPoint>& keypoints);

  // idx img --> idx point cloud  
  int idxIMG_to_idxXYZRGB( int& y, int& x, int& size_img_y, int& size_img_x );

  // compute center of mass
  Eigen::Vector3f compute_center_of_mass_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<int> idx);

  // get index vector Cloud
  vector<vector<int>> get_index_vector();

  // Compute Cross Covariance Matrix from xyz data
  Eigen::Matrix3f cross_covariance_matrix(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pcl);

  // function to compute initial values for translation, rotattion
  Eigen::Matrix4f get_transformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pcl);

  double get_overallerror(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl, Eigen::Matrix4f Transformation);

  double get_overallerror_eu(Eigen::Matrix4f& Transformation);

  bool merge_pointclouds(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl, Eigen::Matrix4f Transformation);

  bool clone_pcl(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl);

  void apply_transformation(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl, Eigen::Matrix4f Transformation);

  void print_corpoints(void);

  void DownSample(int method,double scale,PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl);

  std::vector<Feature2D> SIFT_FeatureDetector(const cv::Mat& image_rgb, Mat& descriptors, vector<cv::KeyPoint>& keypoints);

  std::vector<Feature2D> SURF_FeatureDetector(const cv::Mat& image_rgb, Mat& descriptors, vector<cv::KeyPoint>& keypoints);

  std::vector<Feature2D> ORB_FeatureDetector(const cv::Mat& image_rgb, Mat& descriptors, vector<cv::KeyPoint>& keypoints);

  void CorPointsFeatureBased(int Method, const cv::Mat& image_rgb, const cv::Mat& image_depth);

  void FeatureMatcher(int Method, vector<DMatch>& matches, Mat& prev_descriptors, Mat& new_descriptors, const cv::Mat& image_rgb, vector<cv::KeyPoint>& keypoints);

  std::vector<DMatch> FilterMatches(vector<DMatch>& matches, Mat& new_descriptors);

  void DMatch2CoPointsArray(vector<DMatch>& descriptormatches);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud;

  bool FirtIteration = true;
  bool Mapping = false;

  double Lowrange_limit;
  double Highrange_limit;

  int frame_num_offset;
  int frame_num;
  int run_frame_num;

  double MaxDist_NN;
  int NN_SearchType;

  int CorPoints_num;            // Number of Corresponding points 
  double Registration_accuracy_FB;
  double Registration_accuracy_NNS;

  int minCorPoints;

  int Initial_Transformation;

  double maxDepthRange;
  double minDepthRange;

  int CorPointSamMode;

  int DownSampling_mode;
  double DownSampling_scale;

  int FeatureType;

  int MatcherType;
  double Epsilon_minDistFM;
  double Mul_minDistFM;

  bool Feature2DShow;
  bool MatcherShow;
  bool ICPIterationShow;
  bool DebuggLogShow;

  string frame_id;
  string child_frame_id;
  bool Publish_TF;

  vector<vector<int>> idx_xyz;

  Mat prev_descriptors;
  Mat prev_rgb_img;

  Matrix4f prevResultTrans;
  Matrix4f initResultTrans;

  vector<cv::KeyPoint> prev_keypoints;
  std::vector<Feature2D> prev_prod;
  
private:
  
	float fx_d = 525.0;
  float fy_d = 525.0;
  float cx_d = 319.5;
  float cy_d = 239.5;

  Eigen::Vector3f Centroid_A = Vector3f::Zero();
  Eigen::Vector3f Centroid_B = Vector3f::Zero();

    
};

}  // namespace map_recon ...
