#include "MapOptimization.h"
#include "utils/gtsamUtils.h"
#include <pcl/registration/transformation_estimation_svd.h>

void MapOptimization::globalMatchingThread()
{
    rclcpp::Rate rate(globalMatchingFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        if(!system_initialized)
            continue;
        globalMatching();
    }
}

void MapOptimization::globalMatching() {
    static pcl::PointCloud<PointType>::Ptr subTemporalCornerMap;
    static pcl::PointCloud<PointType>::Ptr subTemporalSurfMap;
    static pcl::PointCloud<PointType>::Ptr subTemporalMap;
    static pcl::PointCloud<PointType>::Ptr subPrebuiltMap;
    static pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    static pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;
    static pcl::VoxelGrid<PointType> downSizeFilterCornerForGlobalMatching;
    static pcl::VoxelGrid<PointType> downSizeFilterSurfForGlobalMatching;
    static bool isInitialized = false;

    if(!isInitialized){
        subTemporalCornerMap.reset(new pcl::PointCloud<PointType>());
        subTemporalSurfMap.reset(new pcl::PointCloud<PointType>());
        subTemporalMap.reset(new pcl::PointCloud<PointType>());
        subPrebuiltMap.reset(new pcl::PointCloud<PointType>());

        downSizeFilterCornerForGlobalMatching.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurfForGlobalMatching.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        isInitialized = true;
    }

    {
        std::lock_guard<std::mutex> lock(mtx);
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    }

    int idx = copy_cloudKeyPoses3D->size()-1;
    if(matchedIndexContainer.find(idx) != matchedIndexContainer.end())
        return;

    PointType currentPosition = copy_cloudKeyPoses3D->points[idx];
    PointTypePose currentPose = copy_cloudKeyPoses6D->points[idx];

    //RCLCPP_INFO(rclcpp::get_logger("Debug"), "position, %f %f %f %d", currentPosition.x, currentPosition.y, currentPosition.z, currentPosition.intensity);
    //RCLCPP_INFO(rclcpp::get_logger("Debug"), "pose, %f %f %f %d",currentPose.x, currentPose.y, currentPose.z, currentPose.intensity);

    if (!pcl::isFinite(currentPosition) || !pcl::isFinite(currentPose)) {
        RCLCPP_INFO(rclcpp::get_logger("global matching"), "Invalid position data detected in currentPosition/Pose.");
        return;
    }

    // prebuilt map
    if(useKeyFrame) {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyPosesForGlobalMatching(new pcl::KdTreeFLANN<PointType>());
        kdtreeKeyPosesForGlobalMatching->setInputCloud(kfPrebuilt3D);
        kdtreeKeyPosesForGlobalMatching->radiusSearch(currentPosition, collectKeyframeRange * 3.0f, pointSearchInd,
                                                      pointSearchSqDis);

        //RCLCPP_INFO(rclcpp::get_logger("global matching"), "searched key-poses size : %d", pointSearchInd.size());
        if(pointSearchInd.size() <= 5)
            return;
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

    subPrebuiltMap->clear();
    *subPrebuiltMap = *subCornerPrebuiltMap + *subSurfPrebuiltMap;

    // current sub-map
    {
        std::lock_guard<std::mutex> lock(mtx);
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        int numPoses = copy_cloudKeyPoses3D->size();
        if(numPoses <= 1)
            return;
        for (int i = numPoses - 1; i >= std::max(0, numPoses - numberOfKeyframeForTempolarMap); --i)
            surroundingKeyPoses->push_back(copy_cloudKeyPoses3D->points[i]);

        // extract sub-map
        subTemporalCornerMap->clear();
        subTemporalSurfMap->clear();
        subTemporalMap->clear();
        for (int i = 0; i < (int)surroundingKeyPoses->size(); ++i)
        {
            int thisKeyInd = (int)surroundingKeyPoses->points[i].intensity;
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &copy_cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &copy_cloudKeyPoses6D->points[thisKeyInd]);
            *subTemporalCornerMap += laserCloudCornerTemp;
            *subTemporalSurfMap += laserCloudSurfTemp;
        }
        downSizeFilterCorner.setInputCloud(subTemporalCornerMap);
        downSizeFilterCorner.filter(*subTemporalCornerMap);
        downSizeFilterSurf.setInputCloud(subTemporalSurfMap);
        downSizeFilterSurf.filter(*subTemporalSurfMap);

        *subTemporalMap = *subTemporalCornerMap + *subTemporalSurfMap;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(0.6);
    icp.setMaximumIterations(30);
    icp.setRANSACIterations(0);

    icp.setInputSource(subTemporalMap);
    icp.setInputTarget(subPrebuiltMap);
    pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
    icp.align(*result);

    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    Eigen::Affine3f initialize_affine = pclPointToAffine3f(currentPose);
    Eigen::Affine3f tCorrect = correctionLidarFrame * initialize_affine;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    if(icp.getFitnessScore() <= 1.e-8) // float overflow
        return;

    // RCLCPP_INFO(rclcpp::get_logger("global matching"), "matcing info : %f, %f, %f, %f", x, y, z, icp.getFitnessScore());
    if (icp.hasConverged() && icp.getFitnessScore() < globalMatchingFitnessScore) {
        //RCLCPP_INFO(rclcpp::get_logger("global matching"), "success global matching");
        matchedIndexContainer.insert(idx);
        float rot_var = 0.01f * (std::abs(roll) + std::abs(pitch) + std::abs(yaw));
        float pos_var = icp.getFitnessScore() * 0.1;

        {
            std::lock_guard<std::mutex> lock(mtxGlobalMatching);
            globalMatchingResult.emplace_back(idx, roll, pitch, yaw, x, y, z, pos_var, pos_var);
//            RCLCPP_INFO(rclcpp::get_logger("global matching"), "try global matching : %f", pos_var);
        }
    }else{
//        RCLCPP_INFO(rclcpp::get_logger("global matching"), "try global matching : fail %f", icp.getFitnessScore());

    }

    // clear not_used
    {
        int numPoses = copy_cloudKeyPoses3D->size();

        for(int i=0; i< (numPoses - numberOfKeyframeForTempolarMap - 1); i++){
            cornerCloudKeyFrames[i]->clear();
            surfCloudKeyFrames[i]->clear();
        }

    }

}