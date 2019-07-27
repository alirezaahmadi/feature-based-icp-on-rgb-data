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
#include "map_recon_nodehandler.h"
#include "map_recon.h"

using namespace std;
using namespace message_filters;
using namespace Eigen;

namespace map_recon {

MapReconNodeHandler::MapReconNodeHandler(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
  
  ROS_ERROR("node is runnig....");
  ReadConfig_run(MapManager);
  if (!ReadConfig_map() && !ReadConfig_run(MapManager)){
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  subscriber_depth.subscribe( nodeHandle , depth_image_SubscriberTopic , 1000 );
  subscriber_rgb.subscribe( nodeHandle , rgb_image_SubscriberTopic , 1000 );

  pcl_Publisher_ = nodeHandle_.advertise<pcl::PointCloud<pcl::PointXYZRGB>> (pcl_PublisherTopic, 1);
  map_Publisher_ = nodeHandle_.advertise<pcl::PointCloud<pcl::PointXYZRGB>> (map_PublisherTopic, 1);
  depth_image_Publisher_ = nodeHandle_.advertise<sensor_msgs::Image>(depthImage_PublisherTopic, 1);

  MapManager.map_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  MapManager.prev_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  MapManager.curr_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  MapManager.raw_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  MapManager.prevResultTrans = Matrix4f::Identity();
  switch(MapManager.Initial_Transformation){
    case 0:{      // initialize identity
      MapManager.initResultTrans = Matrix4f::Identity();
      break;
    }case 1:{     // initialize with matrix  (todo ...) 
      MapManager.initResultTrans = Matrix4f::Identity();
      break;
    }case 2:{
      for(int cnt=0; cnt<5; cnt++){
        try{
          kinect_listener.lookupTransform("/openni_depth_frame","/world",   
                                    ros::Time(0), kinect_transform);
          break;
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(3.0).sleep();
        }
      }

      Eigen::Affine3d transform_eigen;
      TransformTFToEigenImpl(kinect_transform,transform_eigen);
      MapManager.initResultTrans(0,0) = static_cast<float>(transform_eigen.rotation()(0,0));
      MapManager.initResultTrans(1,0) = static_cast<float>(transform_eigen.rotation()(1,0));
      MapManager.initResultTrans(2,0) = static_cast<float>(transform_eigen.rotation()(2,0));
      MapManager.initResultTrans(3,0) = 0;

      MapManager.initResultTrans(0,1) = static_cast<float>(transform_eigen.rotation()(0,1));
      MapManager.initResultTrans(1,1) = static_cast<float>(transform_eigen.rotation()(1,1));
      MapManager.initResultTrans(2,1) = static_cast<float>(transform_eigen.rotation()(2,1));
      MapManager.initResultTrans(3,1) = 0;

      MapManager.initResultTrans(0,2) = static_cast<float>(transform_eigen.rotation()(0,2));
      MapManager.initResultTrans(1,2) = static_cast<float>(transform_eigen.rotation()(1,2));
      MapManager.initResultTrans(2,2) = static_cast<float>(transform_eigen.rotation()(2,2));
      MapManager.initResultTrans(3,2) = 0;

      MapManager.initResultTrans(0,3) = static_cast<float>(transform_eigen.translation().x());
      MapManager.initResultTrans(1,3) = static_cast<float>(transform_eigen.translation().y());
      MapManager.initResultTrans(2,3) = static_cast<float>(transform_eigen.translation().z());
      MapManager.initResultTrans(3,3) = 1;
      break;  // get initial ransformation from TF Tree 
    }
    default:{
      MapManager.initResultTrans = Matrix4f::Identity();
    }
  }
  MapManager.prevResultTrans = MapManager.initResultTrans;
  cout << MapManager.prevResultTrans << endl;
  
  // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
  sync_.reset(new sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth ));
  sync_->registerCallback(boost::bind(&map_recon::MapReconNodeHandler::kinect_callback, this, _1, _2));
  ROS_INFO("Successfully launched node.");
}

MapReconNodeHandler::~MapReconNodeHandler() {
}

bool MapReconNodeHandler::ReadConfig_map() {
  if (!nodeHandle_.getParam("rgb_image_SubscriberTopic",
                            rgb_image_SubscriberTopic) ||
      !nodeHandle_.getParam("depth_image_SubscriberTopic",
                            depth_image_SubscriberTopic) ||
      !nodeHandle_.getParam("pcl_PublisherTopic",
                            pcl_PublisherTopic) ||
      !nodeHandle_.getParam("map_PublisherTopic",
                            map_PublisherTopic) ||
      !nodeHandle_.getParam("depthImage_PublisherTopic",
                              depthImage_PublisherTopic))
    return false;
  return true;
}

bool MapReconNodeHandler::ReadConfig_run(MapRecon& Map_manager) {
  cout << "Run parameters loading ..." << endl;
  

  nodeHandle_.param("Initial_Transformation",
                            Map_manager.Initial_Transformation, 0);

  nodeHandle_.param("minDepthRange",
                            Map_manager.minDepthRange, 0.03);
  nodeHandle_.param("maxDepthRange",
                            Map_manager.maxDepthRange, 6.0);

  nodeHandle_.param("Lowrange_limit",
                            Map_manager.Lowrange_limit, 0.0);
  nodeHandle_.param("Highrange_limit",
                            Map_manager.Highrange_limit, 6.0);
  
  nodeHandle_.param("frame_num_offset",
                            Map_manager.frame_num_offset, 10);
  nodeHandle_.param("frame_num",
                            Map_manager.frame_num, 10);
  nodeHandle_.param("CorPoints_num",
                            Map_manager.CorPoints_num, 100);
  nodeHandle_.param("MaxDist_NN",
                            Map_manager.MaxDist_NN, 0.3);
  nodeHandle_.param("NN_SearchType",
                            Map_manager.NN_SearchType, 0);
  nodeHandle_.param("Registration_accuracy_NNS",
                            Map_manager.Registration_accuracy_NNS, 0.0001);

  nodeHandle_.param("DownSampling_mode",
                            Map_manager.DownSampling_mode, 1);
  nodeHandle_.param("DownSampling_scale",
                            Map_manager.DownSampling_scale, 0.1);
  nodeHandle_.param("CorPointSamMode",
                            Map_manager.CorPointSamMode, 0);
  
  nodeHandle_.param("Registration_accuracy_FB",
                            Map_manager.Registration_accuracy_FB, 0.0001);
  nodeHandle_.param("minCorPoints",
                            Map_manager.minCorPoints, 10);
  nodeHandle_.param("FeatureType",
                            Map_manager.FeatureType, 0);
  nodeHandle_.param("MatcherType",
                            Map_manager.MatcherType, 0);
  nodeHandle_.param("Epsilon_minDistFM",
                            Map_manager.Epsilon_minDistFM, 1.0);
  nodeHandle_.param("Mul_minDistFM",
                            Map_manager.Mul_minDistFM, 3.0);

  nodeHandle_.param("DebuggLogShow",
                            Map_manager.DebuggLogShow, false);
  nodeHandle_.param("Feature2DShow",
                            Map_manager.Feature2DShow, false);
  nodeHandle_.param("MatcherShow",
                            Map_manager.MatcherShow, false);
  nodeHandle_.param("ICPIterationShow",
                            Map_manager.ICPIterationShow, false);

  nodeHandle_.getParam("frame_id",
                            Map_manager.frame_id);
  nodeHandle_.getParam("child_frame_id",
                            Map_manager.child_frame_id);
  nodeHandle_.param("Publish_TF",
                            Map_manager.Publish_TF, false);

  Map_manager.idx_xyz.resize(2, vector<int>(Map_manager.CorPoints_num));
  for (int i = 0; i < 2; i++) std::fill(Map_manager.idx_xyz[i].begin(), Map_manager.idx_xyz[i].end(), 0);
  return true;
}

void MapReconNodeHandler::rgb_img_callback(const sensor_msgs::Image::ConstPtr& input_img) {
  ROS_INFO("I received rgb image with height: [%i]", input_img->height);
}

void MapReconNodeHandler::depth_img_callback(const sensor_msgs::Image::ConstPtr& input_img){
   ROS_INFO("I received depth image with height: [%i]", input_img->height);
}

void MapReconNodeHandler::tf_publisher(string frame_id, string child_frame_id, ros::Time time_stamp, Eigen::Matrix4f transformation){
  tf::Vector3 origin;
  origin.setValue(static_cast<double>(transformation(0,3)),static_cast<double>(transformation(1,3)),static_cast<double>(transformation(2,3)));
  
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)), 
        static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)), 
        static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  kinect_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}

void MapReconNodeHandler::TransformTFToEigenImpl(const tf::Transform &t, Eigen::Affine3d &e){
  for(int i=0; i<3; i++){
    e.matrix()(i,3) = t.getOrigin()[i];
    for(int j=0; j<3; j++){
      e.matrix()(i,j) = t.getBasis()[i][j];
    }
  }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++)
    e.matrix()(3, col) = 0;
  e.matrix()(3,3) = 1;
}

void MapReconNodeHandler::kinect_callback(const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::ImageConstPtr& msg_rgb) {
  // Solve all of perception here... 
  //ROS_INFO("depth image with height: [%i], and RGB image with width: [%i]", depth->height,rgb->width);
  cv_bridge::CvImagePtr img_ptr_rgb;
  cv_bridge::CvImagePtr img_ptr_depth;
  try{
    img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }
  try{
    img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb,sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  cv::Mat new_rgb_img(img_ptr_rgb->image.rows, img_ptr_rgb->image.cols, CV_8UC3);
  new_rgb_img = img_ptr_rgb->image;
  cv::Mat new_depth_img(img_ptr_depth->image.rows, img_ptr_depth->image.cols, CV_32FC1);
  new_depth_img = img_ptr_depth->image;
  bool RegOK = false;
  double last_error=0.0;
  Matrix4f ResultTrans;
  ResultTrans = Matrix4f::Identity();
  if(MapManager.frame_num_offset > 0){
    MapManager.frame_num_offset--;
  }else if(MapManager.frame_num > 0 ){           // number of frames to process ... from database
    if(MapManager.DebuggLogShow)cout << "frame_num: " << MapManager.frame_num << endl;
    if(MapManager.CorPointSamMode == 0){

      // img_to_cloud with downsamoling in itself... to do.....
      MapManager.raw_cloud = MapManager.img_to_cloud(new_rgb_img, new_depth_img);
      MapManager.DownSample(MapManager.DownSampling_mode, MapManager.DownSampling_scale
        ,MapManager.curr_cloud, MapManager.raw_cloud); 
      MapManager.apply_transformation(MapManager.curr_cloud, MapManager.curr_cloud,MapManager.prevResultTrans);
      MapManager.clone_pcl(MapManager.map_cloud, MapManager.curr_cloud);
      
      Step1:
      if(FirtIteration){
        MapManager.clone_pcl(MapManager.prev_cloud, MapManager.curr_cloud);
        if(MapManager.DebuggLogShow)cout << "first PCL is added..." << endl;
        FirtIteration = false;
        RegOK = true;
      }else{
        MapManager.get_CorPoints(MapManager.prev_cloud, MapManager.curr_cloud, new_rgb_img, new_depth_img);

        ResultTrans = MapManager.get_transformation(MapManager.prev_cloud,MapManager.curr_cloud);
        MapManager.prevResultTrans = ResultTrans * MapManager.prevResultTrans ;
        //cout << MapManager.prevResultTrans << endl;

        double error = MapManager.get_overallerror_eu(ResultTrans);
        if(abs(last_error - error) > MapManager.Registration_accuracy_NNS){
          MapManager.apply_transformation(MapManager.curr_cloud, MapManager.curr_cloud, ResultTrans);
          //if(MapManager.ICPIterationShow)pcl_Publisher_.publish(MapManager.curr_cloud);
          last_error = error;
          goto Step1;
        }else{
          MapManager.apply_transformation(MapManager.prev_cloud, MapManager.curr_cloud, ResultTrans);
          if(MapManager.DebuggLogShow)cout << "total Number of points: " << MapManager.curr_cloud->points.size() << endl;
          RegOK = true;
        }
      }
      if(RegOK){
        MapManager.frame_num--;
        if(!MapManager.Publish_TF){
          map_Publisher_.publish(MapManager.prev_cloud); 
        }else{
          map_Publisher_.publish(MapManager.map_cloud); 
          tf_publisher(MapManager.frame_id, MapManager.child_frame_id, ros::Time::now() , MapManager.prevResultTrans);
        }
        RegOK = false;
      }
    }else if(MapManager.CorPointSamMode == 1){

      if(FirtIteration){
        MapManager.prev_rgb_img = new_rgb_img.clone();
        switch(MapManager.FeatureType){
          case 0:{      // SIFT based
              MapManager.prev_prod = MapManager.SIFT_FeatureDetector(MapManager.prev_rgb_img, MapManager.prev_descriptors, MapManager.prev_keypoints);
              if(MapManager.DebuggLogShow)cout << "SIFT - initial ..." << endl;
          break;
          }case 1:{     // SURF based
              MapManager.prev_prod = MapManager.SURF_FeatureDetector(MapManager.prev_rgb_img, MapManager.prev_descriptors, MapManager.prev_keypoints);
              if(MapManager.DebuggLogShow)cout << "SURF - initial ..." << endl;
          break;
          }case 2:{     // ORB based
              MapManager.prev_prod = MapManager.ORB_FeatureDetector(MapManager.prev_rgb_img, MapManager.prev_descriptors, MapManager.prev_keypoints);
              if(MapManager.DebuggLogShow)cout << "ORB - initial ..." << endl;  
          break;
          }
        }
        MapManager.curr_cloud = MapManager.keypoints_to_cloud(MapManager.prev_rgb_img, new_depth_img, MapManager.prev_keypoints);
        MapManager.apply_transformation(MapManager.prev_cloud, MapManager.curr_cloud, MapManager.prevResultTrans);
        if(MapManager.DebuggLogShow)cout << "First Iteration -> total Number of points: " << MapManager.prev_cloud->points.size() << endl;
        FirtIteration = false;
        RegOK = true;
      }else{
        MapManager.get_CorPoints(MapManager.prev_cloud, MapManager.curr_cloud, new_rgb_img, new_depth_img);

        ResultTrans = MapManager.get_transformation(MapManager.prev_cloud, MapManager.curr_cloud);
        MapManager.prevResultTrans = ResultTrans * MapManager.prevResultTrans;
        if(MapManager.DebuggLogShow)cout << ResultTrans <<  endl;

        double error = MapManager.get_overallerror_eu(ResultTrans);
        if(abs(last_error - error) > MapManager.Registration_accuracy_FB){
          if(MapManager.DebuggLogShow)cerr << "Reg Error is more then threshold !!: " << error << endl;
        }else{
          // if(!MapManager.Publish_TF){
          //   MapManager.apply_transformation(MapManager.prev_cloud, MapManager.curr_cloud, ResultTrans);
          // }else{
          //   MapManager.apply_transformation(MapManager.prev_cloud, MapManager.curr_cloud, MapManager.prevResultTrans);
          // }
          if(MapManager.DebuggLogShow)cout << "total Number of points: " << MapManager.curr_cloud->points.size() << endl;
          RegOK = true;
        }
      }
      if(RegOK){
        MapManager.frame_num--;
        map_Publisher_.publish(MapManager.curr_cloud); 
        if(MapManager.Publish_TF)tf_publisher(MapManager.frame_id, MapManager.child_frame_id, ros::Time::now() , MapManager.prevResultTrans);
        RegOK = false;
      }
    }
  }
}

}  // namespace map_recon ... 