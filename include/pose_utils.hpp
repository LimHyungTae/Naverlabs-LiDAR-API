
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <string>
#include <experimental/filesystem>
#include <sstream>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>
#include "signal.h"

#ifndef POSE_UTILS_H
#define POSE_UTILS_H

double UNIT_TIMESTAMP_INTERVAL_ = 0.1;
using PointType = pcl::PointXYZ;
using namespace std;

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

namespace pose_utils {
    vector<pair<int, int> > getIdxPairs(int startIdx, int endIdx, int interval) {
        // pair: <src, tgt> idx pair
        vector<pair<int, int> > idx_pairs;
        bool is_initial = true;
        int i = startIdx;
        while (i < endIdx) {
            if (is_initial) {
                idx_pairs.emplace_back(make_pair(-1, i));
                is_initial = false;
            } else {
                idx_pairs.emplace_back(make_pair(i - interval, i));
            }
            i += interval;
        }
        std::cout << "Total " << idx_pairs.size() << " pairs of idx are prepared" << std::endl;
        return idx_pairs;
    }

    vector<pair<int, int> > fetchLoopIdxes(const pcl::PointCloud<pcl::PointXYZ> &poses_cloud,
                                             int interval_boundary = 50, double min_radius = 0.0,
                                             double max_radius = 5.0) {
        // pair: <src, tgt> idx pair
        int N = poses_cloud.size();
        vector<float> debug_dists;
        vector<pair<int, int> > idx_pairs;
        unique_ptr<Eigen::MatrixXi> status(new Eigen::MatrixXi(N, N));

        int NOT_ASSIGNED = 0;
        int ASSIGNED = 1;
        status->fill(NOT_ASSIGNED);

        pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>);
        *src = poses_cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(src);

        int debug_count = 0;
        for (int src_idx = 0; src_idx < N; ++src_idx) {
            std::vector<int> indices;
            std::vector<float> sqr_dist_list;

            auto pt = poses_cloud[src_idx];
            kdtree.radiusSearch(pt, max_radius, indices, sqr_dist_list);
            int M = indices.size();
            for (int m = 0; m < M; ++m) {
                int tgt_idx = indices[m];
                float dist = sqrt(sqr_dist_list[m]);
                if ((tgt_idx == src_idx) || (abs(tgt_idx - src_idx) < interval_boundary) || (dist < min_radius)) {
                    continue;
                } else { // It could be considered as loop!
                    // Check upper triangular
                    if ((*status)(min(src_idx, tgt_idx), max(src_idx, tgt_idx)) == NOT_ASSIGNED) {
                        idx_pairs.emplace_back(make_pair(min(src_idx, tgt_idx), max(src_idx, tgt_idx)));

                        if (dist < min_radius){
                            cout<<"\033[1;33m"<<dist<<endl;
                            throw invalid_argument("Error!\033[0m");
                        }
                        debug_dists.push_back(dist);
                        (*status)(min(src_idx, tgt_idx), max(src_idx, tgt_idx)) = ASSIGNED;
                        ++debug_count;
                    }
                }
            }
        }

        std::cout << "Total " << idx_pairs.size() << " pairs of idx are loaded" << std::endl;
        return idx_pairs;
    }



    vector<pair<int, int> > randomSample(const vector<pair<int, int> > &allPairs, int numSelected) {
        int N = allPairs.size();
        if (N < numSelected){
            return allPairs;
        }
        vector<pair<int, int> > pairs_sampled;
        pairs_sampled.reserve(numSelected);

        if (numSelected > allPairs.size()){
            return allPairs;
        }
        vector<int> occupancy_check(N, 0);

        srand(1);
        for (int i = 0; i < numSelected; ++i) {
            int idxSelected = rand() % N;
            while (occupancy_check[idxSelected] || !(idxSelected)) { // To avoid idxSelected <- 0
                idxSelected = rand() % N;
            }
            occupancy_check[idxSelected] = 1;
            pairs_sampled.emplace_back(allPairs[idxSelected]);
        }

        std::cout << "Total " << pairs_sampled.size() << " pairs of idx are sampled among " << allPairs.size() << " data." << std::endl;

        return pairs_sampled;
    }

    struct Pose3 {
        Eigen::Translation3f position;
        Eigen::Quaternionf orientation;

        Pose3(Eigen::Matrix4f &tf4x4) :
                position(Eigen::Translation3f(tf4x4.topRightCorner(3, 1))),
                orientation(Eigen::Quaternionf(tf4x4.block<3, 3>(0, 0))) {};

        Pose3(vector<double> &vec) :

                position(Eigen::Translation3f(vec[0], vec[1], vec[2])),
                orientation(Eigen::Quaternionf(vec[3], vec[4], vec[5], vec[6])) {};

        Pose3(Eigen::Translation3f &tf, Eigen::Quaternionf &q) :
                position(tf),
                orientation(q) {};
    };

    double calcTranslationError(Eigen::Matrix4f &rel4x4) {
        double ts_error = 0;
        for (int i = 0; i < 3; ++i) {
            ts_error += rel4x4(i, 3) * rel4x4(i, 3);
        }
        ts_error = sqrt(ts_error);
        return ts_error;
    }

    // http://www.boris-belousov.net/2016/12/01/quat-dist/
    double calcRotationError(Eigen::Matrix4f &rel4x4) {
        Eigen::Matrix3f rel_rot = rel4x4.block<3, 3>(0, 0);
        double rot_error = acos((rel_rot.trace() - 1) / 2);
        return rot_error;
    }

    template<typename T>
    void voxelize(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, pcl::PointCloud<T> &dst, double voxelSize) {
        static pcl::VoxelGrid<T> voxel_filter;
        voxel_filter.setInputCloud(srcPtr);
        voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        voxel_filter.filter(dst);
    }

    uint64_t str2uint64_t(const string &str) {
        uint64_t value;
        std::istringstream iss(str);
        iss >> value;
        return value;
    }

    void writePose(ofstream &FileObj, int idx, Pose3 &P) {
        FileObj << std::fixed << std::setprecision(8)
                << idx * UNIT_TIMESTAMP_INTERVAL_ << " " << P.position.x() << " " << P.position.y() << " "
                << P.position.z()
                << " " << P.orientation.x() << " " << P.orientation.y() << " " << P.orientation.z() << " "
                << P.orientation.w() << "\n";
        FileObj.close();
    }

    vector<float> splitLine(string input, char delimiter) {
        vector<float> answer;
        stringstream ss(input);
        string temp;

        while (getline(ss, temp, delimiter)) {
            answer.push_back(stof(temp));
        }
        return answer;
    }

    void vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4) {
        for (int idx = 0; idx < 12; ++idx) {
            int i = idx / 4;
            int j = idx % 4;
            tf4x4(i, j) = pose[idx];
        }
    }

    vector<pair<int, int> > loadIndicesFromTxtFile(std::string teaser_failure_txt){
        vector<pair<int, int> > idx_pairs;

        std::ifstream in(teaser_failure_txt);
        if (in.fail()){
            throw invalid_argument("[loadIndicesFromTxtFile]: File does not exist");
        }
        std::string line;

        while (std::getline(in, line)) {
            vector<float> teaser_result = splitLine(line, ' ');
            idx_pairs.emplace_back(make_pair(teaser_result[0], teaser_result[1]));
        }
        return idx_pairs;

    }

    vector<Eigen::Matrix4f> loadRotFromRPY(std::string rpy_path){
        std::ifstream rpy_txt(rpy_path);
        std::string line;
        vector<Eigen::Matrix4f> rot_imus;
        int count = 0;
        while (std::getline(rpy_txt, line)) {
            // output the line
            vector<float> rpy = pose_utils::splitLine(line, ' ');
            float y = rpy[2];
            float p = rpy[1];
            float r = rpy[0];
            Eigen::Matrix4f rzryrx = Eigen::Matrix4f::Identity(); // Crucial!
            Eigen::Matrix4f rz =  Eigen::Matrix4f::Identity();
            Eigen::Matrix4f ry =  Eigen::Matrix4f::Identity();
            Eigen::Matrix4f rx =  Eigen::Matrix4f::Identity();
            rz(0, 0) = cos(y);
            rz(1, 1) = cos(y);
            rz(0, 1) = -sin(y);
            rz(1, 0) = sin(y);

            ry(0, 0) = cos(p);
            ry(2, 2) = cos(p);
            ry(0, 2) = sin(p);
            ry(2, 0) = -sin(p);

            rx(1, 1) = cos(r);
            rx(2, 2) = cos(r);
            rx(1, 2) = -sin(r);
            rx(2, 1) = sin(r);

            static Eigen::Matrix4f imu2lidar;
            imu2lidar << 9.999976e-01, 7.553071e-04, -2.035826e-03, -8.086759e-01,
                    -7.854027e-04, 9.998898e-01, -1.482298e-02, 3.195559e-01,
                    2.024406e-03, 1.482454e-02, 9.998881e-01, -7.997231e-01,
                    0, 0, 0, 1;
            rzryrx = rz * ry * rx * imu2lidar.inverse();
            // Lidar coordinate w.r.t. global coordinate
            rot_imus.emplace_back(rzryrx);
            count++;
        }
        return rot_imus;

    }
// T_w_cam0
}

// save the estimated poses
//  std::ofstream ofs("/tmp/traj.txt");
//  for (const auto& pose : poses) {
//    for (int i = 0; i < 3; i++) {
//      for (int j = 0; j < 4; j++) {
//        if (i || j) {
//          ofs << " ";
//        }
//
//        ofs << pose(i, j);
//      }
//    }
//    ofs << std::endl;
//  }

#endif // POSE_UTILS_H
