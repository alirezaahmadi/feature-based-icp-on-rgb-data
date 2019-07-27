# Map-Recon (A pure c++ and ROS implementation of 3D point-cloud registration with ICP (featured base initial guess))

The results of our study on how depth and RGB images from a RGB-D sensor can be used to build a dense 3D map of the indoor environment using different variants of Iterative Closet Point (ICP) approach based on a C++ implementation is shared with you here. Such dense 3D map can be used in robot navigation and environment segmentation applications. The method incorporated in this paper uses descriptor based features extracted from RGB and depth images as correspondence used in the registration process. Also, the result of a point-to-point registration using the Nearest Neighbor (NN) search approach is compared with other variants too. As all the requirements and implementation are quite similar to Simultaneous Localization and Mapping (SLAM) problem, there have been some efforts to detect the loop closures, followed by pose graph optimization to achieve a globally consistent map. Our implementation is based on C++ under ROS platform. The evaluation of 3D point-cloud registration using a RGB-D camera dataset along the ICP method illuminates the efficiency of descriptor-based matching in contrast with simple NN search approach for more complicated models or with more strange misalignments which all the result are explained in details under evaluation section.

## Problem
Minimizing the distance between two point clouds which can be interpreted as tracking the position of camera or in addition if it integrate each scan into a map representation, goal would be 3D reconstruction of environment which is SLAM.

<div align="center">
	<img src="/images/1.png"  width="150"/>
</div>

## Main Pipe-Line
The aim of this section is to explain general idea about how and in which steps the registration of point clouds and 3D reconstruction of the environments in real-time could be done. The registration algorithm includes different parts which shortly can be named as, finding correspondences, noise removal, rejection of wrong correspondences and finding translation and rotation parameters or the alignment. This process can be called as optimization or searching for alignment parameters which could be addressed with different methods like linear or non-linear least square optimizations through Iterative Closest Point (ICP).

<div align="center">
	<img src="/images/graph.png"  width="400"/>
</div>

### DownSampling

The downsampling is an essential step in 3D data processing, because, point clouds are getting more dense to sample more details. To improve the performance of different algorithms which work with point clouds, the number of points should be reduced in such a way to don’t lose important parts of information

<div align="center">
	<img src="/images/down.png"  width="700"/>
</div>

### Outlier removal 
1. Distant base filter (Correspondence
Rejection)
2. Local density analysis

<div align="center">
	<img src="/images/out.png"  width="600"/>
</div>

## Result(Point-to-Point ICP)

<div align="center">
	<img src="/images/ptp.png"  width="500"/>
</div>

## feature based Registration

<div align="center">
	<img src="/images/feature.png"  width="800"/>
</div>