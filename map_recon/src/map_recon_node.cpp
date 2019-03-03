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
