#include "MapOptimization.h"
#include <pcl/registration/ndt.h>
#include <vector>


void MapOptimization::loadGlobalMap()
{
    std::string saveMapDirectory = std::getenv("HOME") + savePCDDirectory;

    if(useKeyFrame){
        string kfpcName = "keyframePointCloud/";
        if(saveMapDirectory.back() != '/')
            kfpcName = "/keyframePointCloud/";
        string keyframePointCloudDir = saveMapDirectory + kfpcName;

        for (size_t i = 0; ; ++i) {
            std::stringstream cornerSS;
            cornerSS << keyframePointCloudDir << "corner_" << i << ".pcd";
            pcl::PointCloud<PointType>::Ptr cornerCloud(new pcl::PointCloud<PointType>());
            if (pcl::io::loadPCDFile<PointType>(cornerSS.str(), *cornerCloud) == -1)
                break;

            cornerCloudKeyFrames.push_back(cornerCloud);
        }

        for (size_t i = 0; ; ++i) {
            std::stringstream surfSS;
            surfSS << keyframePointCloudDir << "surf_" << i << ".pcd";
            pcl::PointCloud<PointType>::Ptr surfCloud(new pcl::PointCloud<PointType>());
            if (pcl::io::loadPCDFile<PointType>(surfSS.str(), *surfCloud) == -1)
                break;

            surfCloudKeyFrames.push_back(surfCloud);
        }

        std::string trajectoryFile = saveMapDirectory + "/trajectory.pcd";
        if (pcl::io::loadPCDFile<PointType>(trajectoryFile, *cloudKeyPoses3D) == -1) {
            PCL_ERROR("Couldn't read file trajectory.pcd \n");
            return;
        }
        std::cout << "Loaded " << trajectoryFile << " with " << cloudKeyPoses3D->points.size() << " points." << std::endl;

        std::string transformationsFile = saveMapDirectory + "/transformations.pcd";
        if (pcl::io::loadPCDFile<PointTypePose>(transformationsFile, *cloudKeyPoses6D) == -1) {
            PCL_ERROR("Couldn't read file transformations.pcd \n");
            return;
        }
        std::cout << "Loaded " << transformationsFile << " with " << cloudKeyPoses6D->points.size() << " points." << std::endl;
    }

    std::cout << "Map directory : " << saveMapDirectory << std::endl;
    pcl::io::loadPCDFile<PointType>(saveMapDirectory + "CornerMap.pcd", *laserCloudCornerFromMap);
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);

    std::cout << "global map size (Corner) : " << laserCloudCornerFromMap->size() << std::endl;

    pcl::io::loadPCDFile<PointType>(saveMapDirectory + "SurfMap.pcd", *laserCloudSurfFromMap);
    downSizeFilterCorner.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterCorner.filter(*laserCloudSurfFromMapDS);

    std::cout << "global map size (Surface) : " << laserCloudSurfFromMap->size() << std::endl;

    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
    has_global_map = true;

    pcl::PointCloud<PointType>::Ptr combinedCloud(new pcl::PointCloud<PointType>());
    *combinedCloud = *laserCloudCornerFromMap + *laserCloudSurfFromMap;
    sleep(2); // for rviz
    rclcpp::Clock clock;
    publishCloud(pubGlobalMap, combinedCloud, clock.now(), mapFrame); // for select pose to localize in rviz
}


void MapOptimization::initialposeHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msgIn)
{
    if(system_initialized)
        return;
    tf2::Quaternion q(msgIn->pose.pose.orientation.x, msgIn->pose.pose.orientation.y,
                      msgIn->pose.pose.orientation.z, msgIn->pose.pose.orientation.w);
    tf2::Matrix3x3 qm(q);

    double roll, pitch, yaw;
    qm.getRPY(roll, pitch, yaw);

    initialize_pose[0] = roll;
    initialize_pose[1] = pitch;
    initialize_pose[2] = yaw;

    initialize_pose[3] = msgIn->pose.pose.position.x;
    initialize_pose[4] = msgIn->pose.pose.position.y;
    initialize_pose[5] = msgIn->pose.pose.position.z;

    std::cout << "manual initialize position: \n" << msgIn->pose.pose.position.x << ":" << msgIn->pose.pose.position.y << ":" << msgIn->pose.pose.position.z  << std::endl;
    // std::cout << "manual initialize pose: \n" << initialize_pose[3] << "\n" << initialize_pose[4] << "\n" << initialize_pose[5] << "\n"
    //           << initialize_pose[0] << "\n" << initialize_pose[1] << "\n" << initialize_pose[2] << std::endl;

    has_initialize_pose = true;
}

bool MapOptimization::systemInitialize()
{
    if (!has_global_map)
        return false;

    if(localizationMethod == "keyframe")
        keyframeLocalization();

    if(!has_initialize_pose)
    {
        // RCLCPP_WARN(rclcpp::get_logger("globalLocalize"), "need initialize pose from rviz.");
        return false;
    }

    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(3.0);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(1e-4);
    icp.setRANSACIterations(0);

    if(useKeyFrame) {
        PointType currentPose;
        static float keyFrameID = 0;
        keyFrameID--; // minus keyFrameID uses for avoid cache (laserCloudMapContainer)
        currentPose = {initialize_pose[3], initialize_pose[4], initialize_pose[5], keyFrameID};

        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        float search_range_iter = 1;
        do{
            if(search_range_iter > 10)
                RCLCPP_ERROR(rclcpp::get_logger("localization"), "can't search keyframe in 50m");

            kdtreeSurroundingKeyPoses->radiusSearch(currentPose, collectKeyframeRange * search_range_iter, pointSearchInd, pointSearchSqDis);
            search_range_iter++;
        }while(pointSearchInd.size() < 10);

        std::cout << "searched key-poses size : " << pointSearchInd.size() << std::endl;
        for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);

            if(i > 15)
                break;
        }
        extractCloud(surroundingKeyPoses);
    }

    Eigen::Affine3f initialize_affine = trans2Affine3f(initialize_pose);

    pcl::PointCloud<PointType>::Ptr combinedCloudLast(new pcl::PointCloud<PointType>());
    *combinedCloudLast = *laserCloudSurfLast + *laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr transformedCombinedCloudLast(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*combinedCloudLast, *transformedCombinedCloudLast, initialize_affine);
    icp.setInputSource(transformedCombinedCloudLast);

    pcl::PointCloud<PointType>::Ptr combinedCloudMap(new pcl::PointCloud<PointType>());
    *combinedCloudMap = *laserCloudSurfFromMap + *laserCloudCornerFromMap;
    icp.setInputTarget(combinedCloudMap);

    pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
    icp.align(*result);

    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    Eigen::Affine3f tCorrect = correctionLidarFrame * initialize_affine;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    transformTobeMapped[0] = roll;
    transformTobeMapped[1] = pitch;
    transformTobeMapped[2] = yaw;
    transformTobeMapped[3] = x;
    transformTobeMapped[4] = y;
    transformTobeMapped[5] = z;

    if (icp.hasConverged() && icp.getFitnessScore() < localizationFitnessScore)
    {
        RCLCPP_INFO(rclcpp::get_logger("localization"), "initialize pose successful");
        system_initialized = true;

        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("localization"), "initialize pose failed");
        has_initialize_pose = false;
        system_initialized = false;
        return false;
    }
}


void MapOptimization::keyframeLocalization(){
    assert(localizationMethod == "keyframe");
    assert(useKeyFrame);

    if(localizationMethod != "keyframe")
        return;
    if(!useKeyFrame){
        RCLCPP_ERROR(rclcpp::get_logger("localization"), "set useKeyFrame true");
        return;
    }

    static std::vector<std::pair<float, Eigen::Affine3f>> ndtResult;

    std::function<void()> getInitialPose = [&]() {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setResolution(1.0);  // Set the grid resolution
        ndt.setMaximumIterations(10);
        ndt.setTransformationEpsilon(1e-4);
        ndt.setStepSize(0.1);  // Step size for the optimizer


        constexpr int sampling_rate = 5;
        for(int i=0; i < cloudKeyPoses6D->size(); i+=sampling_rate){
            auto& currentPose = cloudKeyPoses6D->at(i);
            auto& currentPosition = cloudKeyPoses3D->at(i);

            pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
            kdtreeSurroundingKeyPoses->radiusSearch(currentPosition, collectKeyframeRange, pointSearchInd, pointSearchSqDis);

            for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
                int id = pointSearchInd[i];
                surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);

                if(i > 10)
                    break;
            }
            extractCloud(surroundingKeyPoses);


            Eigen::Affine3f initialize_affine = pclPointToAffine3f(currentPose);
            pcl::PointCloud<PointType>::Ptr combinedCloudLast(new pcl::PointCloud<PointType>());
            *combinedCloudLast = *laserCloudSurfLast + *laserCloudCornerLast;
            pcl::PointCloud<PointType>::Ptr transformedCombinedCloudLast(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*combinedCloudLast, *transformedCombinedCloudLast, initialize_affine);
            ndt.setInputSource(transformedCombinedCloudLast);

            pcl::PointCloud<PointType>::Ptr combinedCloudMap(new pcl::PointCloud<PointType>());
            *combinedCloudMap = *laserCloudSurfFromMap + *laserCloudCornerFromMap;
            ndt.setInputTarget(combinedCloudMap);

            pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
            ndt.align(*result);

            if(ndt.getFitnessScore() < localizationFitnessScore * 2){
                Eigen::Affine3f correctionLidarFrame;
                correctionLidarFrame = ndt.getFinalTransformation();
                Eigen::Affine3f tCorrect = correctionLidarFrame * initialize_affine;

                ndtResult.emplace_back(ndt.getFitnessScore(), tCorrect);
            }
        }

        std::sort(ndtResult.begin(), ndtResult.end(),
                  [](const std::pair<float, Eigen::Affine3f>& a, const std::pair<float, Eigen::Affine3f>& b) {
                      return a.first < b.first;
                  });

    };


    static int index = -1;
    if(index == -1){
        getInitialPose();
        index = 0;
    }

    if(index < ndtResult.size()){
        has_initialize_pose = true;
        auto [_, transform] = ndtResult[index++];

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);

        initialize_pose[0] = roll;
        initialize_pose[1] = pitch;
        initialize_pose[2] = yaw;
        initialize_pose[3] = x;
        initialize_pose[4] = y;
        initialize_pose[5] = z;
    }else{
        has_initialize_pose = false;
        RCLCPP_ERROR(rclcpp::get_logger("localization"), "can't find initial pose using key-frame");
    }
}
