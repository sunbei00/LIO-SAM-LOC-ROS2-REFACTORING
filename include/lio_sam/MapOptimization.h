//
// Created by root on 11/2/24.
//

#ifndef BUILD_MapOptimization_H
#define BUILD_MapOptimization_H

#include <pcl/kdtree/kdtree_flann.h>
// pcl include kdtree_flann throws error if PCL_NO_PRECOMPILE
// is defined before

#include "utils/ParamServer.h"
#include "utils/pclUtils.h"
#include "utils/utils.h"
#include "utils/pclType.h"

#include "lio_sam_loc/msg/cloud_info.hpp"
#include "lio_sam_loc/srv/save_map.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "geometry_msgs/msg/quaternion_stamped.hpp"

#include <deque>

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

class MapOptimization : public ParamServer {
public: // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;
public: // ros2
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

    rclcpp::Service<lio_sam_loc::srv::SaveMap>::SharedPtr srvSaveMap;
    rclcpp::Subscription<lio_sam_loc::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavFix;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subHeading;

public: // ros2 topic from /lio_sam/featureExtraction node
    lio_sam_loc::msg::CloudInfo cloudInfo;

public: // data

    // mapping keyPose idx to point cloud in lidar frame
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;  // idx : keyIndex -> data : cornerCloud
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;    // idx : keyIndex -> data : surfCloud
    // data exist in lidar frame, so need to call transformPointCloud()

    // keyPose
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;               // 3d keyposes
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;           // 6d keyposes

    // point cloud in current lidar frame
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;       // corner feature from FeatureExtraction
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;         // surf feature from FeatureExtraction
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;     // downsampled corner feature set from FeatureExtraction
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;       // downsampled surf feature set from FeatureExtraction
    int laserCloudCornerLastDSNum = 0;                          // downsampled corner feature's size
    int laserCloudSurfLastDSNum = 0;                            // downsampled surf feature's size
    
    // data using at scan2MapOptimization function
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    std::vector<PointType> laserCloudOriCornerVec;              // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;                // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    // (cache) mapping key-pose to point cloud in world coordinate
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    // point cloud sub-map at current pose
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;    // Map is come from nearby
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;      // Map is come from nearby
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;  // downsampled Map is come from laserCloudCornerFromMap
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;    // downsampled Map is come from laserCloudSurfFromMap

    // kd-tree object which is used for searching point
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;       // KdTree from laserCloudCornerFromMapDS
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;         // kdTree from laserCloudSurfFromMapDS

    // kd-tree object which is used for searching key-pose.
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    // filtering object for down-sampling
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    // time stamp of lidar message.
    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    // current pose, roll pitch yaw x y z
    float transformTobeMapped[6];

    // synchronization at saveMap, publish, main pipeline
    std::mutex mtx;
    // synchronization at mtxLoop
    std::mutex mtxLoopInfo;

    // this state means that optimization result is fail.
    bool isDegenerate = false;

    // for visualization.
    nav_msgs::msg::Path globalPath;

    // it uses imu optimization between predicted pose from IMU and LiDAR
    Eigen::Affine3f incrementalOdometryAffineFront; // predicted pose from IMU
    Eigen::Affine3f incrementalOdometryAffineBack;  // predicted pose from LiDAR

    // broadcast TF for debugging
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    // GPS factor, global matching factor
    bool aLoopIsClosed = false;

    // localization
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalMap;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose;
    bool has_global_map = false;
    bool has_initialize_pose = false;
    bool system_initialized = false;
    float initialize_pose[6];

    // pre-built data (kf means key frame)
    pcl::PointCloud<PointType>::Ptr cornerPrebuiltMap;           // map is come from Prebuilt
    pcl::PointCloud<PointType>::Ptr surfPrebuiltMap;             // map is come from Prebuilt
    pcl::PointCloud<PointType>::Ptr combinedPrebuiltMap;         // map is come from Prebuilt
    pcl::PointCloud<PointType>::Ptr subCornerPrebuiltMap;        // sub map is come from Prebuilt (will be also used by global matching)
    pcl::PointCloud<PointType>::Ptr subSurfPrebuiltMap;          // sub map is come from Prebuilt (will be also used by global matching)
    pcl::PointCloud<PointType>::Ptr kfPrebuilt3D;               // 3d keyposes from Prebuilt
    pcl::PointCloud<PointTypePose>::Ptr kfPrebuilt6D;           // 6d keyposes from Prebuilt
    vector<pcl::PointCloud<PointType>::Ptr> kf2cornerPrebuilt;  // idx : keyIndex -> data : cornerCloud
    vector<pcl::PointCloud<PointType>::Ptr> kf2surfPrebuilt;    // idx : keyIndex -> data : surfCloud
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> mapContainerPrebuilt;

    // globalMatching
    // idx, roll, pitch, yaw, x, y, z, rot_variance, pos_variance
    std::vector<std::tuple<int,float,float,float,float,float,float,float,float>> globalMatchingResult;
    std::set<int> matchedIndexContainer;
    std::mutex mtxGlobalMatching;

    // gps
    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    // gps(navfix)
    pcl::PointCloud<PointTypeXYI>::Ptr gpsKFPrebuilt; // gpsKeyPose (UTM Coordinate), idx : keyIndex -> data {east, north, intensity}
    PointTypeXYI currGPS = {-1, -1, -1};           // east north
    double UTM2SLAMyaw = 0;

public: // methods

    // MOInitializer.cpp
    MapOptimization(const rclcpp::NodeOptions& options);
    void allocateMemory();

    // MapOptimization.cpp
    void gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg);
    void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
    void laserCloudInfoHandler(const lio_sam_loc::msg::CloudInfo::SharedPtr msgIn);
    void updateInitialGuess();
    void extractNearby();
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void extractSurroundingKeyFrames();
    void downsampleCurrentScan();
    void cornerOptimization();
    void surfOptimization();
    void combineOptimizationCoeffs();
    bool LMOptimization(int iterCount);
    void scan2MapOptimization();
    void transformUpdate();
    bool saveFrame();
    void addOdomFactor();
    void addGPSFactor();
    void addGlobalMatchingFactor();
    void saveKeyFramesAndFactor();
    void correctPoses();
    void updatePath(const PointTypePose& pose_in);


    // MOPublish.cpp
    void publishOdometry();
    void publishFrames();
    bool saveMap(std::string destination = "", float resolution = 0.0f);

    // MOVisualize.cpp
    void visualizeGlobalMapThread();
    void publishGlobalMap();

    // MOGlobalMatching.cpp
    void globalMatchingThread();
    void globalMatching();

    // MOLocalization.cpp
    void loadGlobalMap();
    void initialposeHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msgIn);
    bool systemInitialize();
    void keyframeLocalization();
    void gpsLocalization();

};




#endif //BUILD_MapOptimization_H
