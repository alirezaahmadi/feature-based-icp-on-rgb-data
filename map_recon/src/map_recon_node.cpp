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

#include <ros/ros.h>
#include "map_recon_nodehandler.h"
#include "map_recon.h"

#define TEST false

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_recon_node");
  ros::NodeHandle nodeHandle("~");

  map_recon::MapReconNodeHandler MapReconNodeHandler(nodeHandle);

  if(TEST){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    map_recon::MapRecon Map_reg;
    Matrix4f Trans;
    Trans <<  0, 0, 0, 1,
    		      0, 1, 0, 3,
    		      0, 0, 1, 2,
              0, 0, 0, 1;
    cout << "Tranformation applied::" << endl;
    cout << Trans << endl;

    //while(true){
    Map_reg.load_pcl_file("/home/alireza/Desktop/1.pcd", test_cloud);

    Map_reg.apply_transformation(Result_cloud,test_cloud, Trans);

    double last_error=0.0;

    Step4:
    Matrix4f ResultTrans = Map_reg.get_transformation(test_cloud,Result_cloud);

    Map_reg.apply_transformation(Result_cloud, Result_cloud, ResultTrans);

    double error = Map_reg.get_overallerror_eu(ResultTrans);

    if(abs(last_error-error) > 0.0001){
      cout << "Iteration Error: " << error <<  " " << last_error-error << endl;
      last_error = error;
      goto Step4;
    }else{
      cout << "Match Complete..."<< "Iteration Error: " << error << " " << last_error-error << endl;
    }

    test_cloud.reset();
    Result_cloud.reset();
  }
  
  ros::spin();
  return 0;

}
