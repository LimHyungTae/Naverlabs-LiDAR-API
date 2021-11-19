#include "labsloader.hpp"
#include "nvutils.h"

string ABS_DIR, TARGET_LOC, DATE, TARGET_LIDAR, TARGET_ALGORITHM;
int    INIT_IDX, INTERVAL, FREQUENCY;
bool   IS_VOXELIZATION_ON;
float  VOXEL_SIZE;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_lidar");
    ros::NodeHandle nh;
    nh.param<string>("/nvlabs/abs_dir", ABS_DIR, "/media/shapelim/UX960NVMe1/naverlabs/2019-04-16_15-35-46");
    nh.param<string>("/nvlabs/target_lidar", TARGET_LIDAR, "lidar0");

    nh.param<int>("/nvlabs/init_idx", INIT_IDX, 0);
    nh.param<int>("/nvlabs/interval", INTERVAL, 1);
    nh.param<int>("/nvlabs/frequency", FREQUENCY, 10);

    nh.param("/nvlabs/is_voxel_on", IS_VOXELIZATION_ON, false);
    nh.param<float>("/nvlabs/voxelsize", VOXEL_SIZE, 0.1); // Setting target

    if (IS_VOXELIZATION_ON) {
        cout << "\033[1;32mVOXELIZATION: " << VOXEL_SIZE << "\033[0m" << endl;
    }
    std::cout << "\033[1;32mInit idx: " << INIT_IDX << "\033[0m" << endl;
    if (INTERVAL < 1) throw invalid_argument("Interval should be larger than 0");

    // Data parsing
    string pcdPath, hdfPath;
    cout << "\033[1;32mTarget: " << ABS_DIR << "\033[0m" << endl;
    pcdPath = ABS_DIR + "/pointclouds_data";
    hdfPath = ABS_DIR + "/groundtruth.hdf5";
    cout << "Target pcd path: " << pcdPath << endl;
    cout << "Target hdf path: " << hdfPath << endl;

    // Set dataloader for LABS Indoor dataset
    LabsLoader loader = LabsLoader(pcdPath, TARGET_LIDAR, hdfPath, IS_VOXELIZATION_ON, VOXEL_SIZE);
    int        N      = loader.size();
    cout << "Parse complete!" << endl;
    cout << "Total " << N << "data are loaded!" << endl;

    // Set ROS visualization publishers
    ros::Publisher SrcWrtBodyPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/src", 100);
    ros::Publisher SrcWrtWorldPublisher = nh.advertise<sensor_msgs::PointCloud2>("/src_wrt_world", 100);
    ros::Publisher GtPosePublisher = nh.advertise<nav_msgs::Path>("/gtpose", 100);

    ros::Rate LoopRate(FREQUENCY);

    nav_msgs::Path GtPath;
    for (int i = INIT_IDX; i < N; ++i) {
        signal(SIGINT, signal_callback_handler);
        cout << i << " th lidar is published!" << endl;
        /***
         * How to load a pointcloud
         * Point clouds are already transformed w.r.t. body frame
         * That is, pose * cloud -> cloud w.r.t. world frame
         */
        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>);
        *srcCloud = *loader.cloud(i);
        sensor_msgs::PointCloud2 msg = cloud2msg(*srcCloud);
        SrcWrtBodyPublisher.publish(msg);

        /***
         * How to load the i-th pose
         */
        Eigen::Matrix4f pose = loader.pose(i);
        setNavPath(pose, i, GtPath);
        GtPosePublisher.publish(GtPath);

        ros::spinOnce();
        LoopRate.sleep();
    }
    return 0;
}
//    for (int j = 0; j < idxPairs.size(); ++j) {
//        signal(SIGINT, signal_callback_handler);
//
//
//        int src_idx = idxPairs[j].first;
//        int tgt_idx = idxPairs[j].second;
//
//        std::cout<<tgt_idx<<" / "<< N<<" th operation...   | ";
//
//        Eigen::Matrix4f gt_body_src = loader.pose(src_idx);
//        gtBodyTf = loader.pose(tgt_idx);
//        // To set start poistion as origin for convenience
//        gtTfWrtInit = init2origin.inverse() * gtBodyTf;
//        Eigen::Matrix4f src2target_gt = gtBodyTf.inverse() * gt_body_src;
//
//        // Set source (past) and target (current) data
//        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>);
//        pcl::PointCloud<PointType>::Ptr tgtCloud(new pcl::PointCloud<PointType>);
//
//        *srcCloud = *loader.cloud(src_idx);
//        *tgtCloud = *loader.cloud(tgt_idx);
//        float opt_time_taken = 0;

//        double time_taken = (double)(end-start)/CLOCKS_PER_SEC;
//        ofstream estTimeTxt(estTimePath, ios::app);
//        estTimeTxt<<std::fixed<<std::setprecision(5)<<opt_time_taken<<endl;
//        estTimeTxt.close();
//
//        // In terms of pointcloud, src2target is estimated
//        // However, we need the relative pose from source to target. So inverse() is necessary
//        Eigen::Matrix4f relativeTf = src2target.inverse();
//        Eigen::Matrix4f estCurrTfWrtInit = estPrevTfWrtInit * relativeTf;
//
//        // Debug
//        Eigen::VectorXf xyzrpy = eigen2xyzrpy(src2target);
//        cout<<"Reltaive pitch: "<<round(xyzrpy[4] * (180.0 / M_PI) * 10000) / 10000 << " deg | score: "<<  score<<endl;
//        ofstream pitchTxt(pitchOutputPath, ios::app);
//        pitchTxt<<gtTfWrtInit(0, 3)<<" "<<gtTfWrtInit(1, 3)<<" "<< gtTfWrtInit(2, 3)<<" "<< xyzrpy[4] * (180.0 / M_PI)<<endl;
//        pitchTxt.close();
//
//        Eigen::Matrix4f diffPose = src2target_gt.inverse() * src2target;
//        double ts_error = pose_utils::calcTranslationError(diffPose);
//        double rot_error = pose_utils::calcRotationError(diffPose) * 180.0 / M_PI;
//        std::cout << std::fixed << std::setprecision(5) << "\033[1;32m | Error: \033[0m" << ts_error << "m, "
//                  << rot_error << "deg" << std::endl;
//        absPoseTxt.open(absPosePath, std::ios::app);
//        absPoseTxt << std::fixed << std::setprecision(5) << ts_error << " " << rot_error << "\n";
//        absPoseTxt.close();
//
//        // -------------------------------------------
//        // Set msgs and publish them
////        cout<<estCurrTfWrtInit<<endl;
////        cout<<estPath.poses.size()<<endl;
//        setNavPath(estCurrTfWrtInit, tgt_idx, estPath);
//
//        gtP = eigen2geoPose(gtTfWrtInit);
//        estP = eigen2geoPose(estCurrTfWrtInit);
//
//        writePose(gtTxt, tgt_idx, gtP);
//        writePose(estTxt, tgt_idx, estP);
//
//        sensor_msgs::PointCloud2 srcMsg = cloud2msg(*srcCloud);
//        sensor_msgs::PointCloud2 tgtMsg = cloud2msg(*tgtCloud);
//        sensor_msgs::PointCloud2 alignMsg = cloud2msg(aligned);
//        pcl::PointCloud<PointType>::Ptr ptrAligned(new pcl::PointCloud<PointType>);
//        *ptrAligned = aligned;
//
//        // Calc correspondences: corr of G-ICP
////        G_ICP->calcCorrespondenceViaNanoFLANN(ptrAligned, tgtCloud, correspondence);
////        setCorrespondenceMarker(CorrespondenceMarker, ptrAligned, tgtCloud, correspondence, G_ICP->getCorrDist());
//        // corr. of TEASER!!
//        if ( (TARGET_ALGORITHM == "TEASER") || (TARGET_ALGORITHM == "TEASER_w_G_ICP")  || (TARGET_ALGORITHM == "SONNY_w_G_ICP")){
//            std::cout<<"\033[1;32m"<<correspondence_output.size()<<"\033[0m"<<std::endl;
////            setCorrespondenceMarker(CorrespondenceMarker, srcCloud, tgtCloud, correspondence_output);
//            CorrespondencePublisher.publish(CorrespondenceMarker);
//
//            pcl::PointCloud<PointType> src_match, tgt_match, src_inliers, tgt_inliers;
//            TeaserSolver->getMatchedResults(src_match, tgt_match, src_inliers, tgt_inliers);
//            sensor_msgs::PointCloud2 SMMsg = cloud2msg(src_match);
//            sensor_msgs::PointCloud2 TMMsg = cloud2msg(tgt_match);
//            sensor_msgs::PointCloud2 SIMsg = cloud2msg(src_inliers);
//            sensor_msgs::PointCloud2 TIMsg = cloud2msg(tgt_inliers);
//            SrcMPublisher.publish(SMMsg);
//            TgtMPublisher.publish(TMMsg);
//            SrcIPublisher.publish(SIMsg);
//            TgtIPublisher.publish(TIMsg);
//        }
//        SrcPublisher.publish(srcMsg);
//        TgtPublisher.publish(tgtMsg);
//        AlignPublisher.publish(alignMsg);
//
//        ESTPosePublisher.publish(estPath);
//
//
//        estPrevTfWrtInit = estCurrTfWrtInit;
//        ros::spinOnce();
//        loop_rate.sleep();
////        loop_rate.sleep();
//        if ((tgt_idx == STOP_IDX) & (STOP_IDX > 0)){
//            while (ros::ok()){
//                SrcPublisher.publish(srcMsg);
//                TgtPublisher.publish(tgtMsg);
//                AlignPublisher.publish(alignMsg);
//
//                GTPosePublisher.publish(gtPath);
//                ESTPosePublisher.publish(estPath);
//
//                CorrespondencePublisher.publish(CorrespondenceMarker);
//
//                estPrevTfWrtInit = estCurrTfWrtInit;
//                ros::spinOnce();
//                loop_rate.sleep();
//            }
//        }
//    }

//    while (ros::ok()){
//        GTPosePublisher.publish(gtPath);
//        ESTPosePublisher.publish(estPath);
//
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//    return 0;
//}
