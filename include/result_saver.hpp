
#include <vector>
#include <string>
#include <experimental/filesystem>
#include <fstream>
#include <boost/format.hpp>

using PointType = pcl::PointXYZ;
#ifndef RESULT_SAVER_H
#define RESULT_SAVER_H
#define PCD_NAME_DELIMITER "_"

using namespace std;

// Only available for lidar0

class ResultSaver {
public:
    ResultSaver(const std::string &abs_dir, const std::string &alg_name, const int interval,
                const std::string mode = "seq") :
            abs_dir_(abs_dir), alg_name_(alg_name), interval_(interval) {
        std::cout << "ABS dir to save results: " << abs_dir << std::endl;

        if (mode == "seq") {

            time_taken_path_ = (boost::format("%s/%s_%d_time.txt") % abs_dir_ % alg_name_ % to_string(interval_)).str();
            clearFile(time_taken_path_);

            gt_traj_path_ = (boost::format("%s/gt_%d_%s.txt") % abs_dir_ % to_string(interval_) % alg_name_).str();
            clearFile(gt_traj_path_);

            est_traj_path_ = (boost::format("%s/est_%s_%d.txt") % abs_dir_ % alg_name_ % to_string(interval_)).str();
            clearFile(est_traj_path_);

            pitch_error_path_ = (boost::format("%s/%s_%d_pitch_errors.txt") % abs_dir % alg_name_ %
                                 to_string(interval_)).str();
            clearFile(pitch_error_path_);

            rel_pose_path_ = (boost::format("%s/%s_%d_abs_errors.txt") % abs_dir_ % alg_name_ %
                              to_string(interval_)).str();
            clearFile(rel_pose_path_);
        } else if (mode == "loop" || mode == "manual") {
//            time_taken_path_ = (boost::format("%s/%s_time.txt") % abs_dir_ % alg_name_ % to_string(interval_)).str();
//            clearFile(time_taken_path_);

            rel_pose_path_ = (boost::format("%s/%s_abs_errors.txt") % abs_dir_ % alg_name_).str();
            clearFile(rel_pose_path_);
        } else {
            throw invalid_argument("Mode is wrong");
        }

    }

    ResultSaver() {}
    ~ResultSaver() {}

    void clearFile(const std::string &path) {
        ofstream file_obj(path);
        file_obj.close();
    }

    void writePose(int idx, const geometry_msgs::Pose &p, const std::string &target) {
        std::ofstream FileObj;
        if (target == "gt") {
            FileObj.open(gt_traj_path_, std::ios::app);
        } else if (target == "est") {
            FileObj.open(est_traj_path_, std::ios::app);
        } else {
            throw invalid_argument((boost::format("Wrong target is coming check your target:%s") % target).str());
        }
        FileObj << std::fixed << std::setprecision(8)
                << idx * UNIT_TIMESTAMP_INTERVAL << " " << p.position.x << " " << p.position.y << " " << p.position.z
                << " " << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w
                << "\n";
        FileObj.close();
    }

    void writeTime(double time_taken) {
        std::ofstream FileObj(time_taken_path_, std::ios::app);
        FileObj << std::fixed << std::setprecision(5) << time_taken << "\n";
        FileObj.close();
    }

    void writePitch(double pitch_error) {
        std::ofstream FileObj(pitch_error_path_, std::ios::app);
        FileObj << std::fixed << std::setprecision(5) << pitch_error << "\n";
        FileObj.close();
    }

    void writeRelErrorForCheckingDegeneracy(int src_idx, int tgt_idx, double trans_error, double rot_error) {
        std::ofstream FileObj(rel_pose_path_, std::ios::app);
        FileObj << std::fixed << std::setprecision(5) << src_idx << " " << tgt_idx << " " << trans_error << " "
                << rot_error << "\n";
        FileObj.close();
    }

    void writeRelErrorForCheckingDegeneracy(int src_idx, int tgt_idx, double ts_gt, double rot_gt,
                                            double trans_error, double rot_error,
                                            int num_max_cliques, int num_mc_inliers, int num_mc_quasi_inliers,
                                            int num_rot_inliers, int num_final_inliers) {
        std::ofstream FileObj(rel_pose_path_, std::ios::app);
        FileObj << std::fixed << std::setprecision(5) << src_idx << " " << tgt_idx << " " << ts_gt << " " << rot_gt
                << " " << trans_error << " " << rot_error << " " << num_max_cliques << " " << num_mc_inliers << " "
                << num_mc_quasi_inliers << " " << num_rot_inliers << " " << num_final_inliers << "\n";
        FileObj.close();
    }

    void writeRelError(double trans_error, double rot_error) {
        std::ofstream FileObj(rel_pose_path_, std::ios::app);
        FileObj << std::fixed << std::setprecision(5) << trans_error << " " << rot_error << "\n";
        FileObj.close();
    }

    void writeRelError(double abs_trans_error, double abs_rot_error, double rel_trans_error, double rel_rot_error) {
        std::ofstream FileObj(rel_pose_path_, std::ios::app);
        FileObj << std::fixed << std::setprecision(5) << abs_trans_error << " " << abs_rot_error << " "
                << rel_trans_error << " " << rel_rot_error << "\n";
        FileObj.close();
    }

private:
    int         interval_;
    std::string abs_dir_;
    std::string alg_name_;

    std::string gt_traj_path_;
    std::string est_traj_path_;
    std::string pitch_error_path_;
    std::string rel_pose_path_;

    std::string time_taken_path_;
};

#endif  // FAST_GICP_KITTILOADER_H
