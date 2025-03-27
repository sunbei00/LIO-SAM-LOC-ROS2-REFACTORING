#include "MapOptimization.h"
#include "utils/CoordinateTransformationSolver.h"
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

        std::string gpsFile = saveMapDirectory + "/gps.pcd";
        if (pcl::io::loadPCDFile<PointTypeXYI>(gpsFile, *gpsKFPrebuilt) == -1) {
            PCL_ERROR("Couldn't read file gps.pcd \n");
            return;
        }
        std::cout << "Loaded " << gpsFile << " with " << gpsKFPrebuilt->points.size() << " points." << std::endl;

//        for(int i=0; i< gpsKFPrebuilt->size(); i++){
//            std::cout << gpsKFPrebuilt->at(i).x << ", " << gpsKFPrebuilt->at(i).y << ", " << gpsKFPrebuilt->at(i).intensity << std::endl;
//        }

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
    publishCloud(pubGlobalMap, cornerPrebuiltMap, clock.now(), mapFrame); // for select pose to localize in rviz
}

bool MapOptimization::systemInitialize()
{
    // RCLCPP_INFO(rclcpp::get_logger("localization"), "Test01");
    if (!has_global_map)
        return false;

    if(localizationMethod == "keyframe")
        keyframeLocalization();

    if(localizationMethod == "gps")
        gpsLocalization();

    if(!has_initialize_pose)
    {
        //RCLCPP_WARN(rclcpp::get_logger("globalLocalize"), "need initialize pose from rviz.");
        return false;
    }

    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(100);
    icp.setRANSACIterations(10);

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
        }while(pointSearchInd.size() < 20);

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


void MapOptimization::gpsLocalization() {
    assert(localizationMethod == "gps");
    assert(useKeyFrame);

    if(!(localizationMethod == "gps"))
        return;

    if(!useKeyFrame){
        RCLCPP_ERROR(rclcpp::get_logger("localization"), "set useKeyFrame true");
        return;
    }
    if(currGPS.intensity == -1)
        return;
    PointTypeXYI gpsData = currGPS;

    CoordinateTransformationSolver solver;
    UTM2SLAMyaw = solver.solve(gpsKFPrebuilt, kfPrebuilt3D);
    //UTM2SLAMyaw = solver.solveClosedForm(gpsKFPrebuilt, kfPrebuilt3D);



    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    float search_range_iter = 1;
    do {
        if (search_range_iter > 10) {
            RCLCPP_ERROR(rclcpp::get_logger("localization"), "can't search keyframe in 50m");
            break;
        }

        float search_radius = collectKeyframeRange * search_range_iter;
        pointSearchInd.clear();
        pointSearchSqDis.clear();

        for (size_t i = 0; i < gpsKFPrebuilt->points.size(); ++i) {
            float dx = gpsKFPrebuilt->points[i].x - gpsData.x;
            float dy = gpsKFPrebuilt->points[i].y - gpsData.y;
            float dist_sq = dx * dx + dy * dy;

//            RCLCPP_ERROR(rclcpp::get_logger("localization"), "Test02");
            int kfIdx = (int)gpsKFPrebuilt->points[i].intensity;
//            RCLCPP_ERROR(rclcpp::get_logger("localization"), "Test03");
            auto kfPose = kfPrebuilt3D->at(kfIdx);
            if (dist_sq <= search_radius * search_radius) { // 반경 내에 있는지 확인
                RCLCPP_INFO(rclcpp::get_logger("localization"), "distance : %f, UTMx : %f, SLAMx: %f, dx : %f,  UTMy : %f, SLAMy: %f, dy : %f", std::sqrt(dist_sq), gpsKFPrebuilt->points[i].x, kfPose.x, dx, gpsKFPrebuilt->points[i].y, kfPose.y, dy );
                pointSearchInd.push_back(i);
                pointSearchSqDis.push_back(dist_sq);
            }
        }
        search_range_iter++;
    } while (pointSearchInd.size() < 10);

    PointTypePose currentPose;
    currentPose.x = 0;
    currentPose.y = 0;
    currentPose.z = 0;
    currentPose.roll = 0;
    currentPose.pitch = 0;
    currentPose.yaw = 0;

    float weightSum = 0.0;
    currentPose.x = 0.0;
    currentPose.y = 0.0;
    currentPose.z = 0.0;

    for(int i = 0; i < pointSearchInd.size(); i++){
        int idx = pointSearchInd.at(i);
        int kfIdx = (int)gpsKFPrebuilt->points[idx].intensity;

        float dx = gpsData.x - gpsKFPrebuilt->points[idx].x;
        float dy = gpsData.y - gpsKFPrebuilt->points[idx].y;

        float distance = sqrt(dx * dx + dy * dy);
        float weight = 1.0 / (distance + 1e-6);  // 작은 값을 더해 0으로 나누는 문제 방지

        float cosTheta = cos(UTM2SLAMyaw);
        float sinTheta = sin(UTM2SLAMyaw);

        float rotatedDx = cosTheta * dx - sinTheta * dy;
        float rotatedDy = sinTheta * dx + cosTheta * dy;

        auto kfPose = kfPrebuilt3D->at(kfIdx);

        RCLCPP_INFO(rclcpp::get_logger("localization"), "(gps based) estimated initial pose - fx: %f, kfx: %f, dx: %f, fy: %f, kfy: %f, dy: %f",
                    kfPose.x + rotatedDx, kfPose.x, rotatedDx, kfPose.y + rotatedDy, kfPose.y, rotatedDy);

        // 가중치를 곱한 값 계산
        currentPose.x += weight * (kfPose.x + rotatedDx);
        currentPose.y += weight * (kfPose.y + rotatedDy);
        currentPose.z += weight * kfPose.z;

        weightSum += weight;
    }

    if (weightSum > 0) {
        currentPose.x /= weightSum;
        currentPose.y /= weightSum;
        currentPose.z /= weightSum;
    }


//    RCLCPP_INFO(rclcpp::get_logger("localization"), "(gps based)initial pose : %f, %f, %f",currentPose.x, currentPose.y, currentPose.z);


    pcl::PointCloud<PointType>::Ptr cornerMap;
    cornerMap.reset(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfMap;
    surfMap.reset(new pcl::PointCloud<PointType>());

    // extract sub-map
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int idx = pointSearchInd.at(i);
        int thisKeyInd = (int)gpsKFPrebuilt->points[idx].intensity;
        if (mapContainerPrebuilt.find(thisKeyInd) != mapContainerPrebuilt.end())
        {
            *cornerMap += mapContainerPrebuilt[thisKeyInd].first;
            *surfMap   += mapContainerPrebuilt[thisKeyInd].second;
        } else {
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(kf2cornerPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(kf2surfPrebuilt[thisKeyInd],  &kfPrebuilt6D->points[thisKeyInd]);

            *cornerMap += laserCloudCornerTemp;
            *surfMap   += laserCloudSurfTemp;

            mapContainerPrebuilt[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
    }
    std::vector<std::pair<float, Eigen::Affine3f>> ndtResult;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(30);
//    icp.setTransformationEpsilon(1e-4);
//    icp.setEuclideanFitnessEpsilon(1e-4);
    icp.setRANSACIterations(0);

//    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
//    ndt.setResolution(0.1);
//    ndt.setMaximumIterations(100);
//    ndt.setTransformationEpsilon(1e-4);
//
//    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
//    gicp.setMaximumIterations(1000);
////    gicp.setTransformationEpsilon(1e-4);
////    gicp.setEuclideanFitnessEpsilon(1e-6);
//    gicp.setMaxCorrespondenceDistance(2.0);

// currentPose의 yaw 값은 이미 radian 단위입니다.
// Coarse-and-Fine heading search (coarse: 22.5° ≒ 0.3927 rad 간격, fine: best coarse yaw 주변 세밀 검색)

    // 맵 point cloud 생성 (corner + surface)
    pcl::PointCloud<PointType>::Ptr combinedCloudMap(new pcl::PointCloud<PointType>());
    *combinedCloudMap = *cornerMap + *surfMap;
    icp.setInputTarget(combinedCloudMap);
// Coarse Search: 전체 360° 범위를 22.5°(0.3927 rad) 간격으로 탐색
    std::vector<std::pair<float, Eigen::Affine3f>> coarseResults;
    float coarseStep = 22.5 * M_PI / 180.0;  // 22.5° in radian (≈0.3927)
    int coarseSteps = 16;  // 360° / 22.5° = 16 steps
    float initialYaw = currentPose.yaw;  // 이미 radian 단위

    for (int i = 0; i < coarseSteps; i++) {
        float testYaw = initialYaw + i * coarseStep;
        // testYaw를 [-pi, pi] 범위로 정규화
        if (testYaw > M_PI)
            testYaw -= 2 * M_PI;
        else if (testYaw < -M_PI)
            testYaw += 2 * M_PI;
        currentPose.yaw = testYaw;
        Eigen::Affine3f initialize_affine = pclPointToAffine3f(currentPose);

        // 변환된 마지막 프레임 point cloud 생성
        pcl::PointCloud<PointType>::Ptr combinedCloudLast(new pcl::PointCloud<PointType>());
        *combinedCloudLast = *laserCloudCornerLastDS + *laserCloudSurfLastDS;
        pcl::PointCloud<PointType>::Ptr transformedCombinedCloudLast(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*combinedCloudLast, *transformedCombinedCloudLast, initialize_affine);
        icp.setInputSource(transformedCombinedCloudLast);



        pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
        icp.align(*result);
        Eigen::Matrix4f finalTrans = icp.getFinalTransformation();
        Eigen::Affine3f correctionLidarFrame(finalTrans);
        Eigen::Affine3f tCorrect = correctionLidarFrame * initialize_affine;

        coarseResults.emplace_back(icp.getFitnessScore(), tCorrect);
    }

// Coarse 결과 중 가장 낮은 fitness score를 보인 yaw 선택
    auto bestCoarseIt = std::min_element(coarseResults.begin(), coarseResults.end(),
                                         [](const std::pair<float, Eigen::Affine3f>& a,
                                            const std::pair<float, Eigen::Affine3f>& b) {
                                             return a.first < b.first;
                                         });
    Eigen::Affine3f bestCoarseTransform = bestCoarseIt->second;
    float bestCoarseYaw;
    {
        float bestCoarseScore = bestCoarseIt->first;
        float x,y,z;
        float roll, pitch;
        pcl::getTranslationAndEulerAngles(bestCoarseTransform, x, y, z, roll, pitch, bestCoarseYaw);
//            RCLCPP_INFO(rclcpp::get_logger("localization"), "coarse initial pose : %f, %f, %f, %f, %f, %f, %f", x, y, z, roll, pitch, bestCoarseYaw, bestCoarseScore);
    }

    icp.setMaxCorrespondenceDistance(0.3);
// Fine Search: bestCoarseYaw 주변에서 ±(coarseStep/2) 범위를 더 세밀하게 탐색
    std::vector<std::pair<float, Eigen::Affine3f>> fineResults;
    float fineRange = coarseStep / 2;  // ±0.19635 rad 범위
    float fineStep = fineRange / 4;      // 약 0.04909 rad씩 세분화 (총 9단계, 중심 포함)
    int fineSteps = 9;

    for (int i = 0; i < fineSteps; i++) {
        float offset = -2 * fineStep + i * fineStep;
        float testYaw = bestCoarseYaw + offset;
        // testYaw 정규화
        if (testYaw > M_PI)
            testYaw -= 2 * M_PI;
        else if (testYaw < -M_PI)
            testYaw += 2 * M_PI;
        currentPose.yaw = testYaw;
        Eigen::Affine3f initialize_affine = pclPointToAffine3f(currentPose);

        pcl::PointCloud<PointType>::Ptr combinedCloudLast(new pcl::PointCloud<PointType>());
        *combinedCloudLast = *laserCloudCornerLastDS + *laserCloudSurfLastDS;
        pcl::PointCloud<PointType>::Ptr transformedCombinedCloudLast(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*combinedCloudLast, *transformedCombinedCloudLast, initialize_affine);
        icp.setInputSource(transformedCombinedCloudLast);

        pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
        icp.align(*result);
        Eigen::Matrix4f finalTrans = icp.getFinalTransformation();
        Eigen::Affine3f correctionLidarFrame(finalTrans);
        Eigen::Affine3f tCorrect = correctionLidarFrame * initialize_affine;

        fineResults.emplace_back(icp.getFitnessScore(), tCorrect);
    }

// Fine 결과 중 가장 낮은 fitness score를 보인 결과 선택
    auto bestFineIt = std::min_element(fineResults.begin(), fineResults.end(),
                                       [](const std::pair<float, Eigen::Affine3f>& a,
                                          const std::pair<float, Eigen::Affine3f>& b) {
                                           return a.first < b.first;
                                       });
    float bestFineScore = bestFineIt->first;
    Eigen::Affine3f bestFineTransform = bestFineIt->second;

    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(bestFineTransform, x, y, z, roll, pitch, yaw);
    initialize_pose[0] = roll;
    initialize_pose[1] = pitch;
    initialize_pose[2] = yaw;
    initialize_pose[3] = x;
    initialize_pose[4] = y;
    initialize_pose[5] = z;
    RCLCPP_INFO(rclcpp::get_logger("localization"), "final initial pose : %f, %f, %f, %f, %f, %f, %f",
                x, y, z, roll, pitch, yaw, bestFineScore);

    has_initialize_pose = true;

}