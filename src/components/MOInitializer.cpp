#include "MapOptimization.h"
#include "utils/QoS.h"

MapOptimization::MapOptimization(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_MapOptimization", options)
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/trajectory", 1);
    pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_global", 1);
    pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry", qos);
    pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry_incremental", qos);
    pubPath = create_publisher<nav_msgs::msg::Path>("lio_sam/mapping/path", 1);
    br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    subCloud = create_subscription<lio_sam_loc::msg::CloudInfo>(
            "lio_sam/feature/cloud_info", qos,
            std::bind(&MapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
    subGPS = create_subscription<nav_msgs::msg::Odometry>(
            gpsTopic, 200,
            std::bind(&MapOptimization::gpsHandler, this, std::placeholders::_1));

    auto saveMapService = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<lio_sam_loc::srv::SaveMap::Request> req, std::shared_ptr<lio_sam_loc::srv::SaveMap::Response> res) -> void {
        (void)request_header;
        res->success = saveMap(req->destination, req->resolution);
    };

    subNavFix = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            navFixTopic, 10, std::bind(&MapOptimization::navSatFixCallback, this, std::placeholders::_1));


    srvSaveMap = create_service<lio_sam_loc::srv::SaveMap>("lio_sam/save_map", saveMapService);
    pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

    pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_local", 1);
    pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
    pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    // localization ------------------------------------------------
    sub_initial_pose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", qos,
            std::bind(&MapOptimization::initialposeHandler, this, std::placeholders::_1));
    pubGlobalMap = create_publisher<sensor_msgs::msg::PointCloud2>("/lio_sam/mapping/cloud_registered", 1);
    // ------------------------------------------------------------

    allocateMemory();

    // localization ------------------------------------------------
    loadGlobalMap();
    // -------------------------------------------------------------
}

void MapOptimization::allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    cornerPrebuiltMap.reset(new pcl::PointCloud<PointType>());
    surfPrebuiltMap.reset(new pcl::PointCloud<PointType>());
    subCornerPrebuiltMap.reset(new pcl::PointCloud<PointType>());
    subSurfPrebuiltMap.reset(new pcl::PointCloud<PointType>());
    combinedPrebuiltMap.reset(new pcl::PointCloud<PointType>());
    kfPrebuilt3D.reset(new pcl::PointCloud<PointType>());
    kfPrebuilt6D.reset(new pcl::PointCloud<PointTypePose>());

    gpsKFPrebuilt.reset(new pcl::PointCloud<PointTypeXYI>());

    for (int i = 0; i < 6; ++i)
        transformTobeMapped[i] = 0;
}

