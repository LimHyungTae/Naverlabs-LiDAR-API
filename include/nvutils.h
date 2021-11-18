#ifndef NVUTILS_H
#define NVUTILS_H

#include <map>

#include <H5Cpp.h>
#include <boost/multi_array.hpp>
#include <experimental/filesystem>
#include <algorithm>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <yaml.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <signal.h>

//using namespace fs = std::filesystem;
using namespace std::experimental::filesystem::v1;
using namespace std;
using PointType = pcl::PointXYZ;

#define PCD_NAME_DELIMITER "_"
#define LARGE_ENOUGH 30000
#define MAP_CLOUD_LARGE_ENOUGH 3000000

#define MARKER_SCALE 3
#define SCORE_MARKER_SCALE 0.7
#define MARKER_LOC 10
#define UNIT_TIMESTAMP_INTERVAL 0.1
#define FEATURE_EXTRACTION_USING_FPFH 0
#define FEATURE_EXTRACTION_USING_SPINNET 1

struct sensorInfos {
    vector<string> ids = {"lidar0", "lidar1"};
    vector<Pose3> sensorPoses = {Pose3({0.000000, 0.000000, 0.400000, 0.000000, 0.707107, -0.707107, 0.000000}),
                                 Pose3({-0.017810, 0.002077, 0.853309, 0.507853, 0.494743, -0.501624,
                                        -0.495670})}; // x,y,z,qw,qx,qy,qz
};

struct Pose3 {
    Eigen::Translation3f position;
    Eigen::Quaternionf orientation;

    Pose3(const Eigen::Matrix4f &tf4x4) :
            position(Eigen::Translation3f(tf4x4.topRightCorner(3, 1))),
            orientation(Eigen::Quaternionf(tf4x4.block<3, 3>(0, 0))) {};

    Pose3(const vector<double> &vec) :

            position(Eigen::Translation3f(vec[0], vec[1], vec[2])),
            orientation(Eigen::Quaternionf(vec[3], vec[4], vec[5], vec[6])) {};

    Pose3(const Eigen::Translation3f &tf, const Eigen::Quaternionf &q) :
            position(tf),
            orientation(q) {};
};

//void readAllHdf5(const string& strHdfFileName, std::map<string, map<uint64_t, gtsam::Pose3> >& lidarPosesMap);
//void readAllTimeStamps(const vector<string>& lidarPcdNames, vector<uint64_t>& timeStamps);

// Conversion
Eigen::Matrix3d eigenf2eigend(const Eigen::Matrix3f &eigen3x3);

Eigen::Matrix4d eigenf2eigend(const Eigen::Matrix4f &eigen4x4);

Eigen::Matrix3f eigend2eigenf(const Eigen::Matrix3d &eigen3x3);

Eigen::Matrix4f eigend2eigenf(const Eigen::Matrix4d &eigen4x4);

uint64_t str2uint64_t(const string &str);

Eigen::Matrix4f pose2eigen(const Pose3 &pose);

Eigen::VectorXf eigen2xyzrpy(const Eigen::Matrix4f &eigenPose);

geometry_msgs::Pose eigen2geoPose(const Eigen::Matrix4f pose);

tf::Matrix3x3 getRotMat(const Eigen::Matrix4f &eigenPose);

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg) {
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg, cloudresult);
    return cloudresult;
}

template<typename T>
void cloudmsg2cloudptr(sensor_msgs::PointCloud2 cloudmsg, boost::shared_ptr<pcl::PointCloud<T> > cloudPtr) {
    pcl::fromROSMsg(cloudmsg, *cloudPtr);
}

void setNavPath(const Eigen::Matrix4f &pose, const int &idx, nav_msgs::Path &path);

// It is for TEASER++
template<typename T>
void pc2eigen(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, Eigen::Matrix<double, 3, Eigen::Dynamic> &eigenXYZ) {
    int N = srcPtr->points.size();
    for (int i = 0; i < N; ++i) {
        eigenXYZ.col(i) << srcPtr->points[i].x, srcPtr->points[i].y, srcPtr->points[i].z;
    }
}

template<typename T>
void voxelize(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, pcl::PointCloud<T> &dst, double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(dst);
}

template<typename T>
void voxelize(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, boost::shared_ptr<pcl::PointCloud<T> > dstPtr,
              double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(*dstPtr);
}

void setTextMarker(visualization_msgs::Marker &marker);

void setPositionMarker(visualization_msgs::Marker &marker, geometry_msgs::Pose &pose);

void setMarkerColor(visualization_msgs::Marker &marker, float score,
                    float lowestBoundary, float middleBoundary, float highestBoundary);

void setCorrespondenceMarker(visualization_msgs::Marker &marker,
                             pcl::PointCloud<PointType>::Ptr ptrSrc, pcl::PointCloud<PointType>::Ptr ptrTgt,
                             std::vector<std::pair<int, float> > &correspondence, float distThreshold);

void setCorrespondenceMarker(visualization_msgs::Marker& marker,
                             pcl::PointCloud<PointType>& src_matched, pcl::PointCloud<PointType>& tgt_matched,
                             float thickness=2, std::vector<float> rgb_color={1.0, 1.0, 1.0});

void setCorrespondenceMarker(visualization_msgs::Marker &marker,
                             pcl::PointCloud<PointType>::Ptr ptrSrc, pcl::PointCloud<PointType>::Ptr ptrTgt,
                             std::vector<std::pair<int, int> > &correspondence,
                             float thickness,
                             std::vector<float> rgb_color={1.0, 1.0, 0.0});

//int loadCloudWrtBody(string& absPath, Eigen::Matrix4f& lidar2body, pcl::PointCloud<PointType>::Ptr dstCloud);
void writePose(ofstream &FileObj, int idx, geometry_msgs::Pose P);

void writePose(ofstream &FileObj, int srcIdx, int tgtIdx, double score, geometry_msgs::Pose P);
//Eigen::Matrix4f loadBodyTf4x4(const map<uint64_t, gtsam::Pose3>& poseMap, const Eigen::Matrix4f& lidar2body, uint64_t keyTs);

void pcl2teaser(const pcl::PointCloud<PointType> &pcl_raw, teaser::PointCloud &cloud);

void pcl2eigen(const pcl::PointCloud<PointType> &pcl_raw, Eigen::Matrix<double, 3, Eigen::Dynamic> &cloud);

void eigen2pcl(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src, pcl::PointCloud<PointType> &cloud);

int calQuadrant(PointType& p);
#endif