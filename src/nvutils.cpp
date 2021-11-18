#include <tools/nvutils.h>

using namespace std;


Eigen::Matrix3d eigenf2eigend(const Eigen::Matrix3f& eigen3x3)
{
    Eigen::Matrix3d result;
    for(int y = 0; y < 3; y++)
    {
        for(int x = 0; x < 3; x ++)
        {
            result(y,x) = eigen3x3(y,x);
        }
    }
    return result;
}
Eigen::Matrix4d eigenf2eigend(const Eigen::Matrix4f& eigen4x4)
{
    Eigen::Matrix4d result;
    for(int y = 0; y < 4; y++)
    {
        for(int x = 0; x < 4; x ++)
        {
            result(y,x) = eigen4x4(y,x);
        }
    }
    return result;
}

Eigen::Matrix3f eigend2eigenf(const Eigen::Matrix3d& eigen3x3)
{
    Eigen::Matrix3f result;
    for(int y = 0; y < 3; y++)
    {
        for(int x = 0; x < 3; x ++)
        {
            result(y,x) = eigen3x3(y,x);
        }
    }
    return result;
}

Eigen::Matrix4f eigend2eigenf(const Eigen::Matrix4d& eigen4x4)
{
    Eigen::Matrix4f result;
    for(int y = 0; y < 4; y++)
    {
        for(int x = 0; x < 4; x ++)
        {
            result(y,x) = eigen4x4(y,x);
        }
    }
    return result;
}

uint64_t str2uint64_t(const string& str){
    uint64_t value;
    std::istringstream iss(str);
    iss >> value;
    return value;
}

Eigen::Matrix4f pose2eigen(const Pose3& pose){
    auto R = pose.orientation.toRotationMatrix();
    Eigen::Matrix4f tf4x4 = Eigen::Matrix4f::Identity();
    tf4x4.topLeftCorner<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
    tf4x4.topRightCorner(3, 1) = pose.position.vector();
    return tf4x4;
}

geometry_msgs::Pose eigen2geoPose(const Eigen::Matrix4f pose)
{
    geometry_msgs::Pose geoPose;

    tf::Matrix3x3 m;
    m.setValue((double)pose(0,0), (double)pose(0,1), (double)pose(0,2),
               (double)pose(1,0), (double)pose(1,1), (double)pose(1,2),
               (double)pose(2,0), (double)pose(2,1), (double)pose(2,2));

    tf::Quaternion q;
    m.getRotation(q);
    geoPose.orientation.x = q.getX();
    geoPose.orientation.y = q.getY();
    geoPose.orientation.z = q.getZ();
    geoPose.orientation.w = q.getW();

    geoPose.position.x = pose(0,3);
    geoPose.position.y = pose(1,3);
    geoPose.position.z = pose(2,3);

    return geoPose;
}

tf::Matrix3x3 getRotMat(const Eigen::Matrix4f& eigenPose){
    tf::Matrix3x3 rotMat;
    Eigen::Matrix4d pose = eigenf2eigend(eigenPose);
    rotMat.setValue(pose(0,0), pose(0,1), pose(0,2),
                    pose(1,0), pose(1,1), pose(1,2),
                    pose(2,0), pose(2,1), pose(2,2));
    return rotMat;

}

Eigen::VectorXf eigen2xyzrpy(const Eigen::Matrix4f& eigenPose)
{
    Eigen::VectorXf result(6);
    tf::Matrix3x3 rotMat = getRotMat(eigenPose);

    double r, p, y;
    rotMat.getRPY(r, p, y);

    result[0] = eigenPose(0, 3);
    result[1] = eigenPose(1, 3);
    result[2] = eigenPose(2, 3);
    result[3] = r;
    result[4] = p;
    result[5] = y;

    return result;
}


void setNavPath(const Eigen::Matrix4f& pose, const int& idx, nav_msgs::Path& path){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.seq = idx;
    pose_stamped.header.frame_id = "/map";
    pose_stamped.pose = eigen2geoPose(pose);

    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
}


void setTextMarker(visualization_msgs::Marker& marker){
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = MARKER_LOC;
    marker.pose.position.y = MARKER_LOC;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = MARKER_SCALE;
    marker.scale.y = MARKER_SCALE;
    marker.scale.z = MARKER_SCALE;
    marker.color.a = 1.0; // Don't forget to set the alpha!
}

void setPositionMarker(visualization_msgs::Marker& marker, geometry_msgs::Pose& pose){
    static int markerId = 0;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = markerId++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.position.x;
    marker.pose.position.y = pose.position.y;
    marker.pose.position.z = pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = SCORE_MARKER_SCALE;
    marker.scale.y = SCORE_MARKER_SCALE;
    marker.scale.z = SCORE_MARKER_SCALE;
    marker.color.a = 1.0; // Don't forget to set the alpha!
}

void setMarkerColor(visualization_msgs::Marker& marker, float score, float lowestBoundary, float middleBoundary, float highestBoundary){
    if (score > highestBoundary){
        marker.color.r = 0.2;
        marker.color.g = 0.0;
        marker.color.b = 0.2;
    }else if (score > middleBoundary){
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }else if (score > lowestBoundary){
        marker.color.r = 1.0;
        marker.color.g = 0.85;
        marker.color.b = 1.0;
    }else{
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
}

void setCorrespondenceMarker(visualization_msgs::Marker& marker,
                             pcl::PointCloud<PointType>::Ptr ptrSrc, pcl::PointCloud<PointType>::Ptr ptrTgt,
                             std::vector<std::pair<int, float> >& correspondence, float distThreshold){
    if (!marker.points.empty()) marker.points.clear();
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = LARGE_ENOUGH; // To avoid overlap
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!

    geometry_msgs::Point srcP;
    geometry_msgs::Point tgtP;

    assert (ptrSrc->points.size() == correspondence.size());

    for (int idx = 0; idx < ptrSrc->points.size(); ++idx){
        std::pair<int, float>& corr = correspondence[idx];
        if (corr.second > distThreshold){ // 10.0 should be same with the maxDist of registration
            continue;
        }
        PointType& sP = ptrSrc->points[idx];
        PointType& sT = ptrTgt->points[corr.first];
        srcP.x = sP.x;     srcP.y = sP.y;     srcP.z = sP.z;
        tgtP.x = sT.x;     tgtP.y = sT.y;     tgtP.z = sT.z;

        marker.points.emplace_back(srcP);
        marker.points.emplace_back(tgtP);
    }
}


void setCorrespondenceMarker(visualization_msgs::Marker& marker,
                             pcl::PointCloud<PointType>& src_matched, pcl::PointCloud<PointType>& tgt_matched,
                             float thickness, std::vector<float> rgb_color){
    if (!marker.points.empty()) marker.points.clear();
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = LARGE_ENOUGH; // To avoid overlap
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = thickness; // thickness
    marker.color.r = rgb_color[0];
    marker.color.g = rgb_color[1];
    marker.color.b = rgb_color[2];
    marker.color.a = 1.0; // Don't forget to set the alpha!

    geometry_msgs::Point srcP;
    geometry_msgs::Point tgtP;
    assert(src_matched.size() == tgt_matched.size());
    for (int idx = 0; idx < src_matched.size(); ++idx){
        PointType& sP = src_matched[idx];
        PointType& sT = tgt_matched[idx];
        srcP.x = sP.x;     srcP.y = sP.y;     srcP.z = sP.z;
        tgtP.x = sT.x;     tgtP.y = sT.y;     tgtP.z = sT.z;

        marker.points.emplace_back(srcP);
        marker.points.emplace_back(tgtP);
    }
}

void setCorrespondenceMarker(visualization_msgs::Marker& marker,
                             pcl::PointCloud<PointType>::Ptr ptrSrc, pcl::PointCloud<PointType>::Ptr ptrTgt,
                             std::vector<std::pair<int, int> >& correspondence,
                             float thickness, std::vector<float> rgb_color){
    if (!marker.points.empty()) marker.points.clear();
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = LARGE_ENOUGH; // To avoid overlap
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = thickness; // thickness
    marker.color.r = rgb_color[0];
    marker.color.g = rgb_color[1];
    marker.color.b = rgb_color[2];
    marker.color.a = 1.0; // Don't forget to set the alpha!

    geometry_msgs::Point srcP;
    geometry_msgs::Point tgtP;
//    if (correspondence.size() == 0){
//        throw invalid_argument("[Corr. Marker] Correspondences should be larger than 0!!!!");
//    }


    for (int idx = 0; idx < correspondence.size(); ++idx){
        std::pair<int, int> &corr = correspondence[idx];
        PointType& sP = ptrSrc->points[corr.first];
        PointType& sT = ptrTgt->points[corr.second];
        srcP.x = sP.x;     srcP.y = sP.y;     srcP.z = sP.z;
        tgtP.x = sT.x;     tgtP.y = sT.y;     tgtP.z = sT.z;

        marker.points.emplace_back(srcP);
        marker.points.emplace_back(tgtP);
    }
}


// For run EVO
void writePose(ofstream& FileObj, int idx, geometry_msgs::Pose P){
    FileObj<< std::fixed << std::setprecision(8)
           << idx * UNIT_TIMESTAMP_INTERVAL<<" "<<P.position.x<<" "<<P.position.y<<" "<<P.position.z
           <<" "<<P.orientation.x<<" "<<P.orientation.y<<" "<<P.orientation.z<<" "<<P.orientation.w<<"\n";
    FileObj.close();
}

// For checking error. P: src2target, s.t. src2target * src -> target
void writePose(ofstream& FileObj, int srcIdx, int tgtIdx, double score, geometry_msgs::Pose P){
    FileObj<< std::fixed << std::setprecision(8)
           << srcIdx<<" "<<tgtIdx<<" "<<score<<" "<<P.position.x<<" "<<P.position.y<<" "<<P.position.z
           <<" "<<P.orientation.x<<" "<<P.orientation.y<<" "<<P.orientation.z<<" "<<P.orientation.w<<"\n";
    FileObj.close();
}

Eigen::Matrix4f loadBodyTf4x4(const map<uint64_t, Pose3>& poseMap, const Eigen::Matrix4f& lidar2body, uint64_t keyTs){
    return pose2eigen(poseMap.find(keyTs)->second) * lidar2body.inverse(); // body w.r.t. map
}

void pcl2teaser(const pcl::PointCloud<PointType>& pcl_raw, teaser::PointCloud& cloud){
    cloud.clear();
    for (const auto &pt: pcl_raw.points){
        cloud.push_back({pt.x, pt.y, pt.z});
    }
}

void pcl2eigen(const pcl::PointCloud<PointType>& pcl_raw, Eigen::Matrix<double, 3, Eigen::Dynamic>& cloud){
    int N = pcl_raw.points.size();
    cloud.resize(3, N);
    for (int i = 0; i < N; ++i) {
        cloud.col(i) << pcl_raw.points[i].x, pcl_raw.points[i].y, pcl_raw.points[i].z;
    }
}

void eigen2pcl(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src, pcl::PointCloud<PointType>& cloud){
    int num_pc = src.cols();
    PointType pt_tmp;
    if (!cloud.empty()) cloud.clear();
    for (int i = 0; i < num_pc; ++i){
        pt_tmp.x = src(0, i);    pt_tmp.y = src(1, i);     pt_tmp.z = src(2, i);
        cloud.points.emplace_back(pt_tmp);
    }
}

int calcQuadrant(PointType& p){
    bool isXPositive = p.x > 0;
    bool isYPositive = p.y > 0;
    if (isXPositive && isYPositive){
        return 1;
    }else if(!(isXPositive) && isYPositive){
        return 2;
    }else if(!(isXPositive) && !(isYPositive)){
        return 3;
    }else{
        return 4;
    }
}
