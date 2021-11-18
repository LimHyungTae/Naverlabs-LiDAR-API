//
// Created by shapelim on 6/1/21.
//
#ifndef DEEP_REGISTRATION_LABLOADER_H
#define DEEP_REGISTRATION_LABLOADER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>
#include <vector>
#include <string>
#include <experimental/filesystem>
#include "pose_utils.hpp"
#include <experimental/filesystem>
#include <gtsam/geometry/Pose3.h>
#include <H5Cpp.h>
#include <boost/multi_array.hpp>

#define NUM_DATA_LARGE_ENOUGH 50000
#define PCD_NAME_DELIMITER "_"
#define ROBOT_BODY_SIZE 0.5

using PointType = pcl::PointXYZ;

using namespace std::experimental::filesystem::v1;
using namespace std;

void parsePCDFileName(const string &fileName, string &lidarName, string &timeStamp) {
    static int len_extension = 4;
    static int len_lidar_name = 6;
    lidarName = fileName.substr(0, fileName.find(PCD_NAME_DELIMITER));
    timeStamp = fileName.substr(len_lidar_name + 1, fileName.length() - (len_lidar_name + len_extension + 1));
}

void readAllPCD(const string &pcdDir, std::map<string, vector<string> > &lidarPcdNamesMap) {
    vector<string> vecLidar0;
    vector<string> vecLidar1;
    vecLidar0.reserve(NUM_DATA_LARGE_ENOUGH);
    vecLidar0.reserve(NUM_DATA_LARGE_ENOUGH);
    if (!lidarPcdNamesMap.empty()) lidarPcdNamesMap.clear();

    int lenDir = pcdDir.length();
    for (const auto &entry : directory_iterator(pcdDir)) {
        string path_name = entry.path();
        string file_name = path_name.substr(lenDir + 1);
        string lidarName, dummy;
        parsePCDFileName(file_name, lidarName, dummy);

        if (lidarName == "lidar0") {
            vecLidar0.push_back(file_name);
        } else if (lidarName == "lidar1") {
            vecLidar1.push_back(file_name);
        } else {
            throw invalid_argument("LiDAR name is weird! Plz check it");
        }
    }
    // Sort is important to set the data in timesatmp order!
    sort(vecLidar0.begin(), vecLidar0.end());
    sort(vecLidar1.begin(), vecLidar1.end());

    std::cout << "lidar0 pcd size: " << vecLidar0.size() << std::endl;
    std::cout << "lidar1 pcd size: " << vecLidar1.size() << std::endl;

    assert(vecLidar0.size() == vecLidar1.size());

    lidarPcdNamesMap.insert(make_pair("lidar0", vecLidar0));
    lidarPcdNamesMap.insert(make_pair("lidar1", vecLidar1));
}
// Only available for lidar0

template<typename T, size_t DIMENSIONS>
void readHdf5(const H5::H5File &file, const string &dataset_name, boost::multi_array<T, DIMENSIONS> &data) {
    H5::DataSet dataset = file.openDataSet(dataset_name);
    H5::DataSpace fspace = dataset.getSpace();
    vector<hsize_t> dims(DIMENSIONS);
    fspace.getSimpleExtentDims(dims.data());
    data.resize(dims);
    dataset.read(data.data(), dataset.getDataType());
}

void readAllHdf5(const string &strHdfFileName, std::map<string, map<uint64_t, pose_utils::Pose3> > &lidarPosesMap) {
    H5::H5File file(strHdfFileName, H5F_ACC_RDONLY);
    if (!lidarPosesMap.empty()) lidarPosesMap.clear();

    vector<std::string> sensorIds = {"lidar0", "lidar1"};
    for (const auto &sensorId: sensorIds) {

        boost::multi_array<uint64_t, 2> stamp_data;
        readHdf5(file, sensorId + "_stamp", stamp_data);

        boost::multi_array<float, 2> pose_data;
        readHdf5(file, sensorId + "_pose", pose_data);

        if (stamp_data.size() != pose_data.size()) {
            cerr << "There exist some error" << endl;
        }

        int nData = stamp_data.size();
        map<uint64_t, pose_utils::Pose3> stamps_with_poses;

        for (int i = 0; i < nData; i++) {
            Eigen::Translation3f ts(pose_data[i][0], pose_data[i][1], pose_data[i][2]);
            Eigen::Quaternionf q(pose_data[i][3], pose_data[i][4],  pose_data[i][5], pose_data[i][6]);

            stamps_with_poses.insert(make_pair(stamp_data[i][0], pose_utils::Pose3(ts, q)));
        }
        lidarPosesMap.insert({sensorId, stamps_with_poses});
    }
    std::cout << "lidar0 pose size: " << lidarPosesMap.find("lidar0")->second.size() << std::endl;
    std::cout << "lidar1 pose size: " << lidarPosesMap.find("lidar1")->second.size() << std::endl;
    assert(lidarPosesMap.find("lidar0")->second.size() == lidarPosesMap.find("lidar1")->second.size());
    std::cout << "Complete to read all hdf5 file" << std::endl;
}

void readAllTimeStamps(const vector<string> &lidarPcdNames, vector<uint64_t> &timeStamps) {
    // Note that it assumes that timestamps btw lidar0 and lidar1 is same
    if (!timeStamps.empty()) timeStamps.clear();

    string lidarFileName, tmpTimeStamp;
    uint64_t keyTs;
    for (const auto &pcdFileName: lidarPcdNames) {
        parsePCDFileName(pcdFileName, lidarFileName, tmpTimeStamp);
        keyTs = pose_utils::str2uint64_t(tmpTimeStamp);
        timeStamps.emplace_back(keyTs);
    }
}

class LabsLoader {
public:
    LabsLoader(std::string &datasetPath, std::string &targetLidar, std::string &hdfPath, bool is_voxelization_on=false, float voxel_size=0.1):
    datasetPath(datasetPath), targetLidar(targetLidar), isVoxelizationOn(is_voxelization_on), voxelSize(voxel_size){

        std::cout << "\033[1;32mTarget lidar: " << targetLidar << "\033[0m" << std::endl;
        lidar0ToBody << 0, -1, 0, 0,
                -1, 0, -0, 0,
                0, 0, -1, 0.4,
                0, 0, 0, 1;

        lidar1ToBody << 0.00536992, 0.00710506, -0.99996, -0.01781,
                -0.999804, 0.0190819, -0.00523349, 0.002077,
                0.019044, 0.999793, 0.00720614, 0.853309,
                0, 0, 0, 1;

        readAllHdf5(hdfPath, lidarPoses);

        readAllPCD(datasetPath, lidarPcdNames);
        readAllTimeStamps(lidarPcdNames.find("lidar0")->second, timeStamps);
        numFrames = lidarPcdNames.find("lidar0")->second.size();
    }

    ~LabsLoader() {}

    size_t size() const { return numFrames; }


    void parsePCDFileName(const string &fileName, string &lidarName, string &timeStamp) {
        static int lenExtension = 4;
        static int lenLidarName = 6;
        lidarName = fileName.substr(0, fileName.find(PCD_NAME_DELIMITER));
        timeStamp = fileName.substr(lenLidarName + 1, fileName.length() - (lenLidarName + lenExtension + 1));
    }

    Eigen::Matrix4f pose(size_t i) const {
        static const map<uint64_t, pose_utils::Pose3> &lidar0PoseMap = lidarPoses.find("lidar0")->second;
        uint64_t keyTs = timeStamps[i];
        Eigen::Matrix4f gtBodyTf = poseToEigen(lidar0PoseMap.find(keyTs)->second) * lidar0ToBody.inverse(); // body w.r.t. map
        return gtBodyTf;
    }

    Eigen::Matrix4f poseToEigen(const pose_utils::Pose3& pose) const {
        auto R = pose.orientation.toRotationMatrix();
        Eigen::Matrix4f tf4x4 = Eigen::Matrix4f::Identity();
        tf4x4.topLeftCorner<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
        tf4x4.topRightCorner(3, 1) = pose.position.vector();
        return tf4x4;
    }


    int loadCloudWrtBody(string &absPath, Eigen::Matrix4f &lidar2body, pcl::PointCloud<PointType>::Ptr dstCloud) {
        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>);
//    std::cout<<"Loading "<< absPath<<"..."<<std::endl;
        if (pcl::io::loadPCDFile<PointType>(absPath, *srcCloud) == -1) {
            PCL_ERROR ("Couldn't read source pcd file! \n");
            return (-1);
        }
        pcl::transformPointCloud(*srcCloud, *dstCloud, lidar2body);
        return 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud(size_t i, bool is_for_ERASOR = false) const {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_load(new pcl::PointCloud<pcl::PointXYZ>());
        std::string filename;
        if ((targetLidar == "lidar0") | (targetLidar == "lidar1")) {
            filename = datasetPath + "/" + lidarPcdNames.find(targetLidar)->second[i];
            if (targetLidar == "lidar0") {
                loadCloudWrtBody(filename, lidar0ToBody, cloud_load, is_for_ERASOR);
            } else if (targetLidar == "lidar1") {
                loadCloudWrtBody(filename, lidar1ToBody, cloud_load, is_for_ERASOR);
            } else { throw invalid_argument("Check your target lidar"); }
            *cloud = *cloud_load;

        } else if (targetLidar == "both") {
            filename = datasetPath + "/" + lidarPcdNames.find("lidar0")->second[i];
            loadCloudWrtBody(filename, lidar0ToBody, cloud_load, is_for_ERASOR);
            *cloud = *cloud_load;

            filename = datasetPath + "/" + lidarPcdNames.find("lidar1")->second[i];
            loadCloudWrtBody(filename, lidar1ToBody, cloud_load, is_for_ERASOR);
            *cloud += *cloud_load;

        } else { throw invalid_argument("Target lidar keyword is not matched"); }

        if (isVoxelizationOn){
            cout<<"Conduct voxelization! -> ";
            pcl::PointCloud<PointType>::Ptr tmpPtr (new pcl::PointCloud<PointType>);
            *tmpPtr = *cloud;
            pose_utils::voxelize(tmpPtr, *cloud, voxelSize);
        }
        return cloud;
    }


    int loadCloudWrtBody(string &absPath, const Eigen::Matrix4f &lidar2body,
                         pcl::PointCloud<PointType>::Ptr dstCloud, bool is_for_ERASOR = true) const {
        // ToDo: is for ERASOR option makes the code dirty I think
        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>);
//    std::cout<<"Loading "<< absPath<<"..."<<std::endl;
        if (pcl::io::loadPCDFile<PointType>(absPath, *srcCloud) == -1) {
            PCL_ERROR ("Couldn't read source pcd file! \n");
            return (-1);
        }

        pcl::transformPointCloud(*srcCloud, *dstCloud, lidar2body);

        // The order should be transform -> filter!!!
        // This is for ERASOR: Preparation the map with out ceilings
        if (is_for_ERASOR) {
            pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>);
            filterNoisyPts(dstCloud, *filteredCloud);
            *dstCloud = *filteredCloud;
        }
        return 0;
    }

    void filterNoisyPts(pcl::PointCloud<PointType>::Ptr &srcCloud, pcl::PointCloud<PointType> &dstCloud) const {
        static float max_dist_square = pow(ROBOT_BODY_SIZE, 2);
        static float Z_MIN = -0.1;
        static float Z_MAX = 2.0;
        dstCloud.clear();
        for (auto const &pt : srcCloud->points) {
            double dist_square = pow(pt.x, 2) + pow(pt.y, 2);
            if ((dist_square > max_dist_square) & (pt.z > Z_MIN) & (pt.z < Z_MAX)) {
                dstCloud.points.emplace_back(pt);
            }
        }
    }

private:
    int numFrames;
    std::string datasetPath;
    std::string targetLidar;
    Eigen::Matrix4f lidar0ToBody;
    Eigen::Matrix4f lidar1ToBody;

    float voxelSize;
    bool isVoxelizationOn;
    std::vector<uint64_t> timeStamps;
    std::map<string, vector<string> > lidarPcdNames;
    std::map<string, map<uint64_t, pose_utils::Pose3> > lidarPoses;

};

#endif //DEEP_REGISTRATION_LABLOADER_H
