#include "MapOptimization.h"
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <vector>
#include <omp.h>


void MapOptimization::loadGlobalMap()
{
    std::string saveMapDirectory = std::getenv("HOME") + savePCDDirectory;

    if(useKeyFrame){
        string kfpcName = "keyframePointCloud/";
        if(saveMapDirectory.back() != '/')
            kfpcName = "/keyframePointCloud/";
        string keyframePointCloudDir = saveMapDirectory + kfpcName;

        std::string trajectoryFile = saveMapDirectory + "/trajectory.pcd";
        if (pcl::io::loadPCDFile<PointType>(trajectoryFile, *kfPrebuilt3D) == -1) {
            PCL_ERROR("Couldn't read file trajectory.pcd \n");
            return;
        }
        std::cout << "Loaded " << trajectoryFile << " with " << kfPrebuilt3D->points.size() << " points." << std::endl;

        std::string transformationsFile = saveMapDirectory + "/transformations.pcd";
        if (pcl::io::loadPCDFile<PointTypePose>(transformationsFile, *kfPrebuilt6D) == -1) {
            PCL_ERROR("Couldn't read file transformations.pcd \n");
            return;
        }
        std::cout << "Loaded " << transformationsFile << " with " << kfPrebuilt6D->points.size() << " points." << std::endl;

        for (size_t i = 0; i < kfPrebuilt6D->points.size(); ++i) {
            std::stringstream cornerSS;
            cornerSS << keyframePointCloudDir << "corner_" << i << ".pcd";
            pcl::PointCloud<PointType>::Ptr cornerCloud(new pcl::PointCloud<PointType>());
            if (pcl::io::loadPCDFile<PointType>(cornerSS.str(), *cornerCloud) == -1)
                break;

            kf2cornerPrebuilt.push_back(cornerCloud);
        }

        for (size_t i = 0;i < kfPrebuilt6D->points.size(); ++i) {
            std::stringstream surfSS;
            surfSS << keyframePointCloudDir << "surf_" << i << ".pcd";
            pcl::PointCloud<PointType>::Ptr surfCloud(new pcl::PointCloud<PointType>());
            if (pcl::io::loadPCDFile<PointType>(surfSS.str(), *surfCloud) == -1)
                break;

            kf2surfPrebuilt.push_back(surfCloud);
        }

    }

//    {
//
//        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
//
//        for (int i = 0; i < (int) kfPrebuilt3D->size(); ++i)
//            surroundingKeyPoses->push_back(kfPrebuilt3D->points[i]);
//
//        // extract sub-map
//        subCornerPrebuiltMap->clear();
//        subSurfPrebuiltMap->clear();
//        for (int i = 0; i < (int)surroundingKeyPoses->size(); ++i)
//        {
//            int thisKeyInd = (int)surroundingKeyPoses->points[i].intensity;
//            if (mapContainerPrebuilt.find(thisKeyInd) != mapContainerPrebuilt.end())
//            {
//                *subCornerPrebuiltMap += mapContainerPrebuilt[thisKeyInd].first;
//                *subSurfPrebuiltMap   += mapContainerPrebuilt[thisKeyInd].second;
//            } else {
//                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(kf2cornerPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);
//                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(kf2surfPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);
//
//                *subCornerPrebuiltMap += laserCloudCornerTemp;
//                *subSurfPrebuiltMap   += laserCloudSurfTemp;
//
//                mapContainerPrebuilt[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
//            }
//        }
//
//        pcl::PointCloud<PointType>::Ptr builtMap(new pcl::PointCloud<PointType>());
//        *builtMap = *subCornerPrebuiltMap + *subSurfPrebuiltMap;
//        sleep(2); // for rviz
//        rclcpp::Clock clock;
//        publishCloud(pubGlobalMap, builtMap, clock.now(), mapFrame);
//    }

    std::cout << "Map directory : " << saveMapDirectory << std::endl;
    pcl::io::loadPCDFile<PointType>(saveMapDirectory + "CornerMap.pcd", *cornerPrebuiltMap);
    downSizeFilterCorner.setInputCloud(cornerPrebuiltMap);
    downSizeFilterCorner.filter(*cornerPrebuiltMap);

    std::cout << "global map size (Corner) : " << cornerPrebuiltMap->size() << std::endl;

    pcl::io::loadPCDFile<PointType>(saveMapDirectory + "SurfMap.pcd", *surfPrebuiltMap);
    downSizeFilterCorner.setInputCloud(surfPrebuiltMap);
    downSizeFilterCorner.filter(*surfPrebuiltMap);

    std::cout << "global map size (Surface) : " << surfPrebuiltMap->size() << std::endl;

    has_global_map = true;

    *combinedPrebuiltMap = *cornerPrebuiltMap + *surfPrebuiltMap;
    sleep(2); // for rviz
    rclcpp::Clock clock;
    publishCloud(pubGlobalMap, combinedPrebuiltMap, clock.now(), mapFrame); // for select pose to localize in rviz
}

bool MapOptimization::systemInitialize()
{
    if (!has_global_map)
        return false;

    if(localizationMethod == "keyframe")
        keyframeLocalization();

    if(!has_initialize_pose)
    {
        //RCLCPP_WARN(rclcpp::get_logger("globalLocalize"), "need initialize pose from rviz.");
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
        keyFrameID--; // minus keyFrameID uses for avoid cache (mapContainerPrebuilt)
        currentPose = {initialize_pose[3], initialize_pose[4], initialize_pose[5], keyFrameID};

        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurroundingKeyPoses->setInputCloud(kfPrebuilt3D); // create kd-tree
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
            surroundingKeyPoses->push_back(kfPrebuilt3D->points[id]);
        }

        // extract sub-map
        subCornerPrebuiltMap->clear();
        subSurfPrebuiltMap->clear();
        for (int i = 0; i < (int)surroundingKeyPoses->size(); ++i)
        {
            int thisKeyInd = (int)surroundingKeyPoses->points[i].intensity;
            if (mapContainerPrebuilt.find(thisKeyInd) != mapContainerPrebuilt.end())
            {
                *subCornerPrebuiltMap += mapContainerPrebuilt[thisKeyInd].first;
                *subSurfPrebuiltMap   += mapContainerPrebuilt[thisKeyInd].second;
            } else {
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(kf2cornerPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(kf2surfPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);

                *subCornerPrebuiltMap += laserCloudCornerTemp;
                *subSurfPrebuiltMap   += laserCloudSurfTemp;

                mapContainerPrebuilt[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }
    } else {
        *subCornerPrebuiltMap = *cornerPrebuiltMap;
        *subSurfPrebuiltMap = *surfPrebuiltMap;
    }

    Eigen::Affine3f initialize_affine = trans2Affine3f(initialize_pose);

    pcl::PointCloud<PointType>::Ptr combinedCloudLast(new pcl::PointCloud<PointType>());
    *combinedCloudLast = *laserCloudCornerLastDS + *laserCloudSurfLastDS;
    pcl::PointCloud<PointType>::Ptr transformedCombinedCloudLast(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*combinedCloudLast, *transformedCombinedCloudLast, initialize_affine);
    icp.setInputSource(transformedCombinedCloudLast);

    pcl::PointCloud<PointType>::Ptr combinedSubMap(new pcl::PointCloud<PointType>());
    *combinedSubMap = *subCornerPrebuiltMap + *subSurfPrebuiltMap;
    icp.setInputTarget(combinedSubMap);

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

    RCLCPP_INFO(rclcpp::get_logger("localization"), "manual initialize position: %f, %f, %f", msgIn->pose.pose.position.x, msgIn->pose.pose.position.y , msgIn->pose.pose.position.z);

    // std::cout << "manual initialize pose: \n" << initialize_pose[3] << "\n" << initialize_pose[4] << "\n" << initialize_pose[5] << "\n"
    //           << initialize_pose[0] << "\n" << initialize_pose[1] << "\n" << initialize_pose[2] << std::endl;

    has_initialize_pose = true;
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

        omp_lock_t mapContainerPrebuiltLock;
        omp_lock_t ndtResultLock;
        omp_init_lock(&mapContainerPrebuiltLock);
        omp_init_lock(&ndtResultLock);
        constexpr int sampling_rate = 5;

        std::vector<pcl::NormalDistributionsTransform<PointType, PointType>> ndts; // ndt objects
        std::vector<pcl::KdTreeFLANN<PointType>> kdTreeFlanns;                     // kdTree objects for searching keyposes in radius range
        std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>> maps; // sub-maps
        std::vector<pcl::PointCloud<PointType>::Ptr> surroundingKeyPosesVector;    // searching result using kdTree

        for(int i=0; i<numberOfCores; i++){
            ndts.push_back(pcl::NormalDistributionsTransform<PointType, PointType>());
            auto& ndt = ndts.back();
            ndt.setResolution(0.8);
            ndt.setMaximumIterations(50);
            ndt.setTransformationEpsilon(1e-4);
            ndt.setStepSize(0.3);

            kdTreeFlanns.push_back(pcl::KdTreeFLANN<PointType>());

            maps.emplace_back(new pcl::PointCloud<PointType>(), new pcl::PointCloud<PointType>());

            surroundingKeyPosesVector.emplace_back(new pcl::PointCloud<PointType>());
        }
        RCLCPP_INFO(rclcpp::get_logger("localization"), "calculate initial pose from keyframe ..");

        #pragma omp parallel for num_threads(numberOfCores)
        for(int i=0; i < kfPrebuilt6D->size(); i+=sampling_rate){
            int thread_id = omp_get_thread_num();
            auto& ndt = ndts[thread_id];
            auto& kdTreeFlann = kdTreeFlanns[thread_id];

            auto& [cornerMap, surfMap] = maps[thread_id];
            cornerMap->clear();
            surfMap->clear();

            auto& surroundingKeyPoses = surroundingKeyPosesVector[thread_id];
            surroundingKeyPoses->clear();

            auto currentPose = kfPrebuilt6D->at(i);
            auto& currentPosition = kfPrebuilt3D->at(i);

            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            kdTreeFlann.setInputCloud(kfPrebuilt3D);
            kdTreeFlann.radiusSearch(currentPosition, collectKeyframeRange, pointSearchInd, pointSearchSqDis);

            for (int i = 0; i < (int) pointSearchInd.size(); ++i)
                surroundingKeyPoses->push_back(kfPrebuilt3D->points[pointSearchInd[i]]);

            // extract sub-map
            for (int i = 0; i < (int)surroundingKeyPoses->size(); ++i)
            {
                int thisKeyInd = (int)surroundingKeyPoses->points[i].intensity;
                if (mapContainerPrebuilt.find(thisKeyInd) != mapContainerPrebuilt.end())
                {
                    *cornerMap += mapContainerPrebuilt[thisKeyInd].first;
                    *surfMap   += mapContainerPrebuilt[thisKeyInd].second;
                } else {
                    pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(kf2cornerPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);
                    pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(kf2surfPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);

                    *cornerMap += laserCloudCornerTemp;
                    *surfMap   += laserCloudSurfTemp;

                    omp_set_lock(&mapContainerPrebuiltLock);
                    mapContainerPrebuilt[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
                    omp_unset_lock(&mapContainerPrebuiltLock);
                }
            }

            // keypose + yaw (90, 180, 270, 360)
            for(int i=0; i < 4; i++){
                currentPose.yaw += 1.5708;
                if(currentPose.yaw > 3.141592)
                    currentPose.yaw -= 3.141592*2;
                Eigen::Affine3f initialize_affine = pclPointToAffine3f(currentPose);
                pcl::PointCloud<PointType>::Ptr combinedCloudLast(new pcl::PointCloud<PointType>());
                *combinedCloudLast = *laserCloudCornerLastDS + *laserCloudSurfLastDS;
                pcl::PointCloud<PointType>::Ptr transformedCombinedCloudLast(new pcl::PointCloud<PointType>());
                pcl::transformPointCloud(*combinedCloudLast, *transformedCombinedCloudLast, initialize_affine);
                ndt.setInputSource(transformedCombinedCloudLast);

                pcl::PointCloud<PointType>::Ptr combinedCloudMap(new pcl::PointCloud<PointType>());
                *combinedCloudMap = *cornerMap + *surfMap;
                ndt.setInputTarget(combinedCloudMap);

                pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
                ndt.align(*result);

                Eigen::Affine3f correctionLidarFrame;
                correctionLidarFrame = ndt.getFinalTransformation();
                Eigen::Affine3f tCorrect = correctionLidarFrame * initialize_affine;

                omp_set_lock(&ndtResultLock);
                ndtResult.emplace_back(ndt.getFitnessScore(), tCorrect);
                omp_unset_lock(&ndtResultLock);
            }
        }
        omp_destroy_lock(&mapContainerPrebuiltLock);
        omp_destroy_lock(&ndtResultLock);

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
        auto [score, transform] = ndtResult[index++];

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);

        initialize_pose[0] = roll;
        initialize_pose[1] = pitch;
        initialize_pose[2] = yaw;
        initialize_pose[3] = x;
        initialize_pose[4] = y;
        initialize_pose[5] = z;
        RCLCPP_INFO(rclcpp::get_logger("localization"), "initial pose : %f, %f, %f, %f, %f, %f, %f", x, y, z, roll, pitch, yaw, score);

    }else{
        has_initialize_pose = false;
        RCLCPP_ERROR(rclcpp::get_logger("localization"), "can't find initial pose using key-frame");
    }
}
