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
#include "map_recon.h"

namespace map_recon {
// calss Constructor
MapRecon::MapRecon() {
    
};
// calss deconstructor
MapRecon::~MapRecon(){};

// void MapRecon::draw_cloud(const std::string &text, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
//     pcl::visualization::CloudViewer viewer(text);
//     viewer.showCloud(cloud);
// }

void MapRecon::load_pcl_file(string address, PointCloud<PointXYZRGB>::Ptr& pcf){

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (address, *pcf) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    std::cout << "Loaded "
            << pcf->width * pcf->height
            << " data points from test_pcd.pcd... "
            << std::endl;
    // for (size_t i = 0; i < pcf->points.size (); ++i)
    // std::cout << "    " << pcf->points[i].x
    //           << " "    << pcf->points[i].y
    //           << " "    << pcf->points[i].z << std::endl;
}

void MapRecon::filter_point_cloud(PointCloud<PointXYZRGB>::Ptr& pc, PointCloud<PointXYZRGB>::Ptr& pc_f, float minZ, float maxZ){
    PassThrough<PointXYZRGB> filt;
    filt.setInputCloud(pc);
    filt.setFilterFieldName("z"); // filter z dimension
    filt.setFilterLimits(minZ, maxZ);
    filt.setKeepOrganized(true); // Important: to keep the "image-structure" after filtering, if not it becomes 1D (and sparse)
    filt.filter(*pc_f);     
}

bool MapRecon::merge_pointclouds(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl, Eigen::Matrix4f Transformation){

    old_pcl->header.frame_id = new_pcl->header.frame_id;
    pcl::PointXYZRGB point;
    Eigen::Matrix3f Rotation;
    Vector3f new_p;
    Vector3f p;
    Vector3f translation;

    translation << Transformation(0,3),
                   Transformation(1,3),
                   Transformation(2,3);

    Rotation << Transformation(0,0),Transformation(0,1),Transformation(0,2),
                Transformation(1,0),Transformation(1,1),Transformation(1,2),
                Transformation(2,0),Transformation(2,1),Transformation(2,2);

    for (int i=0; i<new_pcl->points.size(); i++){
        
        p(0) = new_pcl->points[i].x;
        p(1) = new_pcl->points[i].y;
        p(2) = new_pcl->points[i].z;

        new_p = ((Rotation*(p - Centroid_B)) + Centroid_A) + translation;
        point.x = new_p(0);
        point.y = new_p(1);
        point.z = new_p(2);

        point.rgb = new_pcl->points[i].rgb;

        old_pcl->points.push_back(point); 
    }
    return true;
}

void MapRecon::apply_transformation(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl, Eigen::Matrix4f Transformation) {

    pcl::PointXYZRGB point;
    Eigen::Matrix3f Rotation;
    Eigen::Vector3f p;
    Eigen::Vector3f new_p;
    Eigen::Vector3f Translation;

    new_pcl->header.frame_id = old_pcl->header.frame_id ;

    new_pcl->points.resize(old_pcl->points.size());

    Translation << Transformation(0,3),
                   Transformation(1,3),
                   Transformation(2,3);

    Rotation << Transformation(0,0),Transformation(0,1),Transformation(0,2),
                Transformation(1,0),Transformation(1,1),Transformation(1,2),
                Transformation(2,0),Transformation(2,1),Transformation(2,2);

    for (int i=0; i<old_pcl->points.size(); i++){
        
        p(0) = old_pcl->points[i].x;
        p(1) = old_pcl->points[i].y;
        p(2) = old_pcl->points[i].z;

        new_p = (Rotation * p) + Translation;
        point.x = new_p(0);
        point.y = new_p(1);
        point.z = new_p(2);

        point.rgb = old_pcl->points[i].rgb;

        new_pcl->points[i] = point; 
    }
}

bool MapRecon::clone_pcl(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl){

    new_pcl->header.frame_id = old_pcl->header.frame_id ;
    pcl::PointXYZRGB point;

    new_pcl->points.resize(old_pcl->points.size());

    for (int i=0; i<old_pcl->points.size(); i++){
        
        point.x = old_pcl->points[i].x;
        point.y = old_pcl->points[i].y;
        point.z = old_pcl->points[i].z;

        point.rgb = old_pcl->points[i].rgb;

        new_pcl->points[i] = point;
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MapRecon::img_to_cloud(const cv::Mat& image_rgb, const cv::Mat &image_depth) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    cloud->header.frame_id = this->child_frame_id;

    for (int y=0;y<image_depth.rows;y++){
        for (int x=0;x<image_depth.cols;x++){
            pcl::PointXYZRGB point;
            float range = image_depth.at<float>(y,x);
            if(range> this->maxDepthRange) continue;
            if(!isnan(range) && range > this->Lowrange_limit){
            point.x = (x - cx_d) * range / fx_d;
            point.y = (y - cy_d) * range / fy_d;
            point.z = range;
            
            cv::Vec3b color = image_rgb.at<cv::Vec3b>(y,x);
            uint8_t r = (color[0]);
            uint8_t g = (color[1]);
            uint8_t b = (color[2]);
            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);
            cloud->points.push_back(point);
            }
        }
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MapRecon::keypoints_to_cloud(const cv::Mat& image_rgb, const cv::Mat &image_depth, vector<cv::KeyPoint>& keypoints) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    int RejectedPoints = 0;
    cloud->header.frame_id = this->child_frame_id;
    cout << "keypoints: " << keypoints.size() << endl;
    for (int i=0; i<keypoints.size(); i++){
        pcl::PointXYZRGB point;
        float range = image_depth.at<float>((int)keypoints[i].pt.y, (int)keypoints[i].pt.x);
        //if(!isnan(range) && range > this->Lowrange_limit && range < this->Highrange_limit){
        if(true){
            if(isnan(range))range = 0;
            if(range < this->Lowrange_limit)range = 0;
            if(range > this->Highrange_limit)range = this->maxDepthRange;
            point.x = ((int)keypoints[i].pt.x - cx_d) * range / fx_d;
            point.y = ((int)keypoints[i].pt.y - cy_d) * range / fy_d;
            point.z = range;
            
            cv::Vec3b color = image_rgb.at<cv::Vec3b>((int)keypoints[i].pt.y, (int)keypoints[i].pt.x);
            uint8_t r = (color[0]);
            uint8_t g = (color[1]);
            uint8_t b = (color[2]);
            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);
            cloud->points.push_back(point);
        }
        else{
            RejectedPoints++;
        }
    }
    if(this->DebuggLogShow)cout << "cloud->points: " << cloud->points.size() << " RejectedPoints: " << RejectedPoints << endl;
    return cloud;
}

int MapRecon::idxIMG_to_idxXYZRGB(int& y, int& x, int& size_img_y, int& size_img_x){
    
    // error caption
    if( x > (size_img_x-1) ){
        cerr << "Error in function idxIMG_to_idxXYZRGB: row_idx (x) exceeds matrix dimension" << endl;
    }
    if( y > (size_img_y-1) ){
        cerr << "Error in function idxIMG_to_idxXYZRGB: col_idx (y) exceeds matrix dimension" << endl;
    }

    // Calc idx in cloud ( pcl struct )
    int idx_xyzrgb = x * (size_img_y - 1) + y;
    return idx_xyzrgb;
}

vector<vector<int>> MapRecon::get_index_vector(){
    int size_y = 640; // size image
    int size_x = 480;

    vector<vector<int>> vector_idx_xyzrgb; // empty vector for indices in cloud
    vector_idx_xyzrgb.resize(2);
    // indices of used points in cloud, size of cloud: [640 x 480] --> 307200 points
    for (int i = 1; i < 4; i++) {
        for (int j = 1; j < 3; j++)
        {
            int idx_img_y = (i * (size_y / 5)) - 1;
            int idx_img_x = (j * (size_x / 4)) - 1;
            int idx_xyzrgb = MapRecon::idxIMG_to_idxXYZRGB(idx_img_y, idx_img_x, size_y, size_x);
            vector_idx_xyzrgb[0].push_back(idx_xyzrgb);
            vector_idx_xyzrgb[1].push_back(idx_xyzrgb);
        }
    }

    return vector_idx_xyzrgb;
}

Eigen::Matrix4f MapRecon::get_transformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pcl){
    Eigen::Matrix4f Trans;
    Trans = Matrix4f::Identity();
    if(old_pcl->points.size() == 0 || new_pcl->points.size() == 0){
        cerr << "Pointclouds are  not loaded correctly..!!!!!!" << endl;
        Trans = Matrix4f::Identity();
        return Trans;
    }else if(this->idx_xyz[0].size() < this->minCorPoints){
        cerr << "Not enough CorrespondinPoints reveived-> can't estimate transformation..!!!!!!" << endl;
        Trans = Matrix4f::Identity();
        return Trans;  
    }else{
        Centroid_A = compute_center_of_mass_xyz(old_pcl, this->idx_xyz[0]);
        //cout << "pointcloud A size: " << old_pcl->points.size()  << "\n "<< "Cemtroid_A: \n" << Cemtroid_A << endl;
        Centroid_B = compute_center_of_mass_xyz(new_pcl, this->idx_xyz[1]);
        //cout << "pointcloud B size: " << new_pcl->points.size() << "\n "<< "Cemtroid_B: \n" << Cemtroid_B << endl;
                                        // (1) Compute Cross Covariance Matrix from xyz points
        Eigen::Matrix3f cov_A_B = MapRecon::cross_covariance_matrix( old_pcl, new_pcl);

                                        // (2) compute singular value decomposition (SVD) --> openCV2
                                        // A = USV*
                                        // Singular values --> D (Diagonal Matrix)
                                        // svd.singularValues();
                                        // svd.matrixU();  
                                        // svd.matrixV();
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_A_B, Eigen::ComputeThinU | Eigen::ComputeThinV);
                                        
                                        // (3) compute Rotation Matrix 
        Eigen::Matrix3f Rotation = svd.matrixU() * (svd.matrixV().transpose());

        //cout << "SVD - Rot:" << endl;
        //cout << Rotation << "\n" << endl;

        //cout << "SVD - Rot_inv:" << endl;
        //cout << Rotation.inverse() << "\n" << endl;

                                        // (4) compute Translation Matrix

        Eigen::Vector3f translation = Centroid_A - Rotation * Centroid_B;

        //cout << "SVD - Trans:" << endl;
        //cout << translation << "\n" << endl;

        
        Trans << Rotation(0,0), Rotation(0,1), Rotation(0,2), translation(0),
                 Rotation(1,0), Rotation(1,1), Rotation(1,2), translation(1),
                 Rotation(2,0), Rotation(2,1), Rotation(2,2), translation(2),
                 0,             0,             0,             1; 
    } 
    return Trans;             
}

Eigen::Vector3f MapRecon::compute_center_of_mass_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, std::vector<int> idx){

    Eigen::Vector3d mu_ = Vector3d::Zero();
    Eigen::Vector3f result  = Vector3f::Zero();

    for (size_t i = 0; i < idx.size(); i++)
    {
        mu_(0) += (double)((cloud->points[idx[i]].x<this->maxDepthRange) ? cloud->points[idx[i]].x : this->maxDepthRange);
        mu_(1) += (double)((cloud->points[idx[i]].y<this->maxDepthRange) ? cloud->points[idx[i]].y : this->maxDepthRange);
        mu_(2) += (double)((cloud->points[idx[i]].z<this->maxDepthRange) ? cloud->points[idx[i]].z : this->maxDepthRange);
    }
    mu_(0) = (double)(mu_(0) / (uint)idx.size());
    mu_(1) = (double)(mu_(1) / (uint)idx.size());
    mu_(2) = (double)(mu_(2) / (uint)idx.size());

    result = mu_.cast <float> ();

    return result;
}

Eigen::Matrix3f MapRecon::cross_covariance_matrix(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pcl){
    // initializaton Cross-Covariance Matrix
    Eigen::Matrix3f cov_A_B = Matrix3f::Zero();
    // center of mass cloud A/B

    // Eigen --> Vector 
    Eigen::Vector3f q;
    Eigen::Vector3f p;

    uint SampleNum = this->idx_xyz[0].size();

    // Compute reduced coordinates (with center of mass)
    for (int i = 0; i < SampleNum ; i++){
        // Reducing xyz coordinates with the center of mass

        q(0) = old_pcl->points[this->idx_xyz[0][i]].x - Centroid_A(0); // Cloud A
        q(1) = old_pcl->points[this->idx_xyz[0][i]].y - Centroid_A(1);
        q(2) = old_pcl->points[this->idx_xyz[0][i]].z - Centroid_A(2);

        p(0) = new_pcl->points[this->idx_xyz[1][i]].x - Centroid_B(0); // Cloud B
        p(1) = new_pcl->points[this->idx_xyz[1][i]].y - Centroid_B(1);
        p(2) = new_pcl->points[this->idx_xyz[1][i]].z - Centroid_B(2);

        cov_A_B += q*(p.transpose());
    }

    return cov_A_B;
}

void MapRecon::print_corpoints(void){
    for (int i = 0; i < this->idx_xyz[0].size(); ++i)
    {
        cout << i << ": " <<this->idx_xyz[0][i] << " -> " << this->idx_xyz[1][i] << endl;
    }
}

double MapRecon::get_overallerror(PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl, Eigen::Matrix4f Transformation){
    double error = 0.0;
    pcl::PointXYZRGB point;
    Eigen::Matrix3f Rotation;
    Vector3f p;
    Vector3f q;
    Vector3f V_error;
    Vector3f Translation;

    Rotation << Transformation(0,0),Transformation(0,1),Transformation(0,2),
                Transformation(1,0),Transformation(1,1),Transformation(1,2),
                Transformation(2,0),Transformation(2,1),Transformation(2,2);

    Translation << Transformation(0),
                   Transformation(1),
                   Transformation(2);


    for (int i=0; i<new_pcl->points.size(); i++){
        
        p(0) = new_pcl->points[i].x;
        p(1) = new_pcl->points[i].y;
        p(2) = new_pcl->points[i].z;

        q(0) = old_pcl->points[i].x;
        q(1) = old_pcl->points[i].y;
        q(2) = old_pcl->points[i].z;

        V_error = q - (Rotation * p) - Translation;

        error += pow((V_error.norm()),2);

    }
    return error;
}

double MapRecon::get_overallerror_eu(Eigen::Matrix4f& Transformation){
    double result = 0.0;
    result = pow(Transformation(0,3),2) + pow(Transformation(1,3),2) + pow(Transformation(2,3), 2);
    return sqrt(result);
}

void MapRecon::DownSample(int method,double scale,PointCloud<PointXYZRGB>::Ptr& new_pcl, PointCloud<PointXYZRGB>::Ptr& old_pcl){
    new_pcl->header.frame_id = old_pcl->header.frame_id;
    int pcl_size = old_pcl->points.size();

    if(scale < 1.0 && scale > 0.0){
        int new_pcl_lenght = (int)(old_pcl->points.size() *scale);
        new_pcl->points.resize(new_pcl_lenght);
        vector<uint> indices;
        indices.resize(new_pcl_lenght);
        //std::fill(indices[i].begin(), indices[i].end(), 0);
        switch(method){
            case 0:     // rendom downsampling
                // Gerenrating unique random samples from pcl
                for (int i = 0; i < new_pcl_lenght; i++){
                    uint num=0;
                    while( count(indices.begin(), indices.end(), num = rand() % pcl_size)) ;
                    indices[i] =  num;
                }
                // pushing selected points to new point could
                for (size_t i = 0; i < new_pcl_lenght; i++) {
                    new_pcl->points[i] = old_pcl->points[indices[i]];
                }
            break;
            case 1:     // patern beased downsampling
                        // todo...
            break;
        }
    }else if(scale >= 1.0){
        for (size_t i = 0; i < old_pcl->points.size(); i++) {
            new_pcl->points[i] = old_pcl->points[i];
        }
    }else if(scale <= 0.0){
        cout << "Error in Scale !!!" << endl;
    }
}

double MapRecon::get_distance(PointXYZRGB& P_new, PointXYZRGB& P_old) {
    double result = pow((P_new.x - P_old.x),2) + pow((P_new.y - P_old.y),2) + pow((P_new.z - P_old.z), 2);
    return sqrt(abs(result));
}

uint MapRecon::get_nearest_neighbour(PointXYZRGB& P_new, PointCloud<PointXYZRGB>::Ptr& pcl_old, double& FminDistance) {
    uint minindex = 0;
    double minDistance = 100;
    double tempDistance = 100;
    int i = 0;
    while(true) {
        tempDistance = get_distance(P_new, pcl_old->points[i]);
        if (minDistance > tempDistance) {
            minDistance = tempDistance;
            minindex = i;
        }
        i++;
        if(pcl_old->points.size()-1 < i)break;
    }
    FminDistance = minDistance;
    return minindex;
}

void MapRecon::update_CoPoints_NN(PointCloud<PointXYZRGB>::Ptr& pcl_new,PointCloud<PointXYZRGB>::Ptr& pcl_old) {
    
    switch(this->NN_SearchType){
        case 0:{
            double fMinDist =100.0;
            uint rand_recov =0;
            for (size_t i = 0; i < this->idx_xyz[0].size(); i++){
                this->idx_xyz[0][i] = get_nearest_neighbour(pcl_new->points[this->idx_xyz[1][i]], pcl_old, fMinDist);
                while(fMinDist > this->MaxDist_NN) {
                    rand_recov = (rand() % (uint)(pcl_new->points.size()));
                    this->idx_xyz[0][i] = get_nearest_neighbour(pcl_new->points[rand_recov], pcl_old, fMinDist);
                    if(fMinDist <= this->MaxDist_NN){
                        //if(this->DebuggLogShow)cout << "rand_recov: " << rand_recov << "  " << this->idx_xyz[0][i] << " " << fMinDist << " "<< this->MaxDist_NN << endl;
                        this->idx_xyz[1][i] = rand_recov;
                        break;
                    }
                } 
            }
        break;
        }case 1:{
            vector<Point2f> pointsForSearch; // all 2D points to search from
            for (int i = 0; i < pcl_new->points.size(); ++i)
            {
                Point2f point;
                point.x = pcl_old->points[i].x;
                point.y = pcl_old->points[i].y;
                pointsForSearch.push_back(point);
            }
            for (size_t i = 0; i < this->idx_xyz[0].size(); i++){
                flann::KDTreeIndexParams indexParams;
                flann::Index kdtree(Mat(pointsForSearch).reshape(1), indexParams);
                vector<float> query;
                query.push_back(pcl_new->points[this->idx_xyz[1][i]].x); //the 2D point.x we need to find neighbours in the query
                query.push_back(pcl_new->points[this->idx_xyz[1][i]].y); //the 2D point.y we need to find neighbours in the query
                vector<int> indices;
                vector<float> dists;
                kdtree.radiusSearch(query, indices, dists, 1, 1);
                this->idx_xyz[0][i] = indices[0];
                //cout << "NN indice: "<< indices[0] << endl;
            }
        break;
        }
    }
}

vector<vector<int>> MapRecon::CorPointsPaternBased(uint8_t Method, uint SampleSize, uint pcl_size){
    vector<vector<int>> Samples;
    Samples.resize(2);

    switch(Method){
        case 0:     // random patern
            for (uint i = 0; i < SampleSize; i++)
            {
                uint rand_inp = (rand() % (uint)(pcl_size)); //  size of pointcould !!!
                Samples[0].push_back(rand_inp);
                Samples[1].push_back(rand_inp);
            } 
        break;
        case 1:     // limit number of points with patern
            Samples = get_index_vector();
        break;
    }
    return Samples;
}

void MapRecon::get_CorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pcl, const cv::Mat& image_rgb, const cv::Mat& image_depth){
    if(this->CorPointSamMode == 0){  // NN
        if(this->FirtIteration){
                                        // get Index Vector for points in clou, which corresponds to the imgage indices
            this->idx_xyz = CorPointsPaternBased(0, this->CorPoints_num,(uint)new_pcl->points.size()); // 0: random patern , 1: grid pater 6 points !!!
            this->FirtIteration = false;  
        }else{
                                        // Iterations to update corresponding points with NN search
            update_CoPoints_NN(new_pcl, old_pcl);
            //print_corpoints();
        }
    }else if(this->CorPointSamMode == 1){  // featured base
        if(this->FirtIteration){
                                        // get Index Vector for points in clou, which corresponds to the imgage indices
            CorPointsFeatureBased(this->FeatureType, image_rgb, image_depth); // 0: SIFT, 1: SURF, 2: ORB 
            //print_corpoints();
            //this->FirtIteration = false;  
        }else{ 
            // do we need to iterate ??? I dont think so!! 
        }
    }
}

void MapRecon::CorPointsFeatureBased(int Method, const cv::Mat& image_rgb, const cv::Mat& image_depth){
    std::vector<Feature2D> new_prod;
    Mat new_descriptors;   // Cor points indexes in point cloud, should I downsample the pointcloud or not??
    vector<cv::KeyPoint> new_keypoints;
    vector<DMatch> matches;
    switch(Method){
        case 0:{      // SIFT based
            new_prod = SIFT_FeatureDetector(image_rgb, new_descriptors, new_keypoints);
        break;
        }case 1:{     // SURF based
            new_prod = SURF_FeatureDetector(image_rgb, new_descriptors, new_keypoints);
        break;
        }case 2:{     // ORB based
            new_prod = ORB_FeatureDetector(image_rgb, new_descriptors, new_keypoints);  
        break;
        }
    }

    this->curr_cloud = keypoints_to_cloud(image_rgb, image_depth, new_keypoints);
    
    FeatureMatcher(this->MatcherType, matches, this->prev_descriptors, new_descriptors, image_rgb, new_keypoints);

    this->prev_keypoints = new_keypoints;
    this->prev_rgb_img = image_rgb;
    this->prev_descriptors = new_descriptors;
}

void MapRecon::FeatureMatcher(int Method, vector<DMatch>& matches, Mat& prev_descriptors, Mat& new_descriptors, const cv::Mat& image_rgb, vector<cv::KeyPoint>& keypoints ){
    
    if(this->DebuggLogShow)cout << "FeatureMatcher: "<< prev_descriptors.size() << "  " << new_descriptors.size() << endl; 
    switch(Method){
        case 0:{        // FLANNBased Matcher
            Ptr<DescriptorMatcher> FLANN_matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            FLANN_matcher->match(prev_descriptors, new_descriptors, matches);
            break;
            }
        case 1:{        // BruteForceBased Matcher
            BFMatcher BF_matcher;
            BF_matcher.match(prev_descriptors, new_descriptors, matches);
            break;
        }
    }

    std::vector<DMatch> good_matches = FilterMatches(matches, new_descriptors);

    if(this->MatcherShow){
        //-- Draw matches
        Mat img_matches;
        drawMatches( this->prev_rgb_img, this->prev_keypoints, image_rgb, keypoints, good_matches, img_matches, Scalar::all(-1),
                     Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        imshow("Matches", img_matches );
        cvWaitKey(30);
    }

    if(this->DebuggLogShow)cout << "Number of Matches: " << matches.size() << " Number of Good Matches: " << good_matches.size() << endl;
    DMatch2CoPointsArray(good_matches);
}

std::vector<DMatch> MapRecon::FilterMatches(vector<DMatch>& matches, Mat& new_descriptors){
    double max_dist = 0; double min_dist = 100;
    std::vector< DMatch > _matches;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < matches.size(); i++ )
    { 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    if(this->DebuggLogShow)cout << "max_dist: " << max_dist << " min_dist: " << min_dist << endl;
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    for( int i = 0; i < matches.size(); i++ ) {
        if( matches[i].distance <= this->Mul_minDistFM * min_dist + this->Epsilon_minDistFM)
            _matches.push_back( matches[i]); 
    }
    return _matches;
}    

void MapRecon::DMatch2CoPointsArray(vector<DMatch>& descriptormatches){
    this->idx_xyz[0].resize(descriptormatches.size(),0);
    this->idx_xyz[1].resize(descriptormatches.size(),0);
    for (size_t i =0; i < descriptormatches.size(); i++)
    {
        this->idx_xyz[0][i] = descriptormatches[i].queryIdx;
        this->idx_xyz[1][i] = descriptormatches[i].trainIdx;
        //cout << descriptormatches[i].queryIdx << " -> " << descriptormatches[i].trainIdx << endl;
    }
    // for (size_t i =0; i < descriptormatches.size(); i++)
    // {
    //     cout << i << ": "<< this->idx_xyz[0][i] << " -> " << this->idx_xyz[1][i] << " " <<this->idx_xyz[0].size() <<endl;
    // }
}

std::vector<Feature2D> MapRecon::SIFT_FeatureDetector(const cv::Mat& image_rgb, Mat& descriptors, vector<cv::KeyPoint>& keypoints){

    std::vector<Feature2D> prod;
    Mat descriptorImage;
    cv::Mat image_gray;
    Feature2D tmp{"Empty", 0, 0, 0, 0, {0}};

    cvtColor( image_rgb, image_gray, CV_BGR2GRAY );

    cv::Ptr<xfeatures2d::SIFT> f2d = xfeatures2d::SIFT::create();
    f2d->detectAndCompute(image_gray, noArray(), keypoints, descriptors);


    if(this->Feature2DShow){
        cv::Mat output;
        cv::drawKeypoints(image_gray, keypoints, output);
        cv::imshow("Feature2D", output);
        cvWaitKey(10);
    } 

    // products get loaded
    for (int row = 0; row < descriptors.rows; row++) {
        for (int col = 0; col < descriptors.cols; col++) {
            tmp.SIFT[col] = descriptors.at<float>(row, col);
        }
        tmp.name = "SIFT";
        tmp.frameNum = this->run_frame_num;
        tmp.FeatureNum = row;
        tmp.pose_x = keypoints[row].pt.x; 
        tmp.pose_y = keypoints[row].pt.y;
        prod.push_back(tmp);
    }
    return prod;
}

std::vector<Feature2D> MapRecon::SURF_FeatureDetector(const cv::Mat& image_rgb, Mat& descriptors, vector<cv::KeyPoint>& keypoints){

    std::vector<Feature2D> prod;
    Mat descriptorImage;
    cv::Mat image_gray;
    Feature2D tmp{"Empty", 0, 0, 0, 0, {0}};

    cvtColor(image_rgb, image_gray, CV_BGR2GRAY);

    int minHessian = 400;
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(minHessian);
    detector->detectAndCompute( image_gray, Mat(), keypoints, descriptors);

    if(this->Feature2DShow){
        cv::Mat output;
        cv::drawKeypoints(image_gray, keypoints, output);
        cv::imshow("Feature2D", output);
        cvWaitKey(10);
    }

    // products get loaded
    for (int row = 0; row < descriptors.rows; row++) {
        for (int col = 0; col < descriptors.cols; col++) {
            tmp.SURF[col] = descriptors.at<float>(row, col);
        }
        tmp.name = "SURF";
        tmp.frameNum = this->run_frame_num;
        tmp.FeatureNum = row;
        tmp.pose_x = keypoints[row].pt.x; 
        tmp.pose_y = keypoints[row].pt.y;
        prod.push_back(tmp);
    }
    return prod;
}

std::vector<Feature2D> MapRecon::ORB_FeatureDetector(const cv::Mat& image_rgb, Mat& descriptors, vector<cv::KeyPoint>& keypoints){

    std::vector<Feature2D> prod;
    Mat descriptorImage;
    cv::Mat image_gray;
    Feature2D tmp{"Empty", 0, 0, 0, 0, {0}};

    cvtColor(image_rgb, image_gray, CV_BGR2GRAY);

    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->detectAndCompute(image_gray, cv::Mat(), keypoints, descriptors);

    if(this->Feature2DShow){
        cv::Mat output;
        cv::drawKeypoints(image_gray, keypoints, output);
        cv::imshow("Feature2D", output);
        cvWaitKey(10);
    } 

    // products get loaded
    for (int row = 0; row < descriptors.rows; row++) {
        for (int col = 0; col < descriptors.cols; col++) {
            tmp.ORB[col] = descriptors.at<float>(row, col);
        }
        tmp.name = "ORB";
        tmp.frameNum = this->run_frame_num;
        tmp.FeatureNum = row;
        tmp.pose_x = keypoints[row].pt.x; 
        tmp.pose_y = keypoints[row].pt.y;
        prod.push_back(tmp);
    }
    return prod;
}

} // namespace map_recon ...
