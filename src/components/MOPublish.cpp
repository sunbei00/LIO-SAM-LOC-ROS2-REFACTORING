#include "MapOptimization.h"

void MapOptimization::publishOdometry()
{
    // Publish odometry for ROS (global)
    nav_msgs::msg::Odometry laserOdometryROS;
    laserOdometryROS.header.stamp = timeLaserInfoStamp;
    laserOdometryROS.header.frame_id = odometryFrame;
    laserOdometryROS.child_frame_id = "odom_mapping";
    laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
    laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
    laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    laserOdometryROS.pose.pose.orientation = quat_msg;
    pubLaserOdometryGlobal->publish(laserOdometryROS);

    // Publish TF
    quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
    tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
    tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
    geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
    tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
    trans_odom_to_lidar.child_frame_id = "lidar_register";
    br->sendTransform(trans_odom_to_lidar);

    // Publish odometry for ROS (incremental)
    static bool lastIncreOdomPubFlag = false;
    static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
    static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
    if (lastIncreOdomPubFlag == false)
    {
        lastIncreOdomPubFlag = true;
        laserOdomIncremental = laserOdometryROS;
        increOdomAffine = trans2Affine3f(transformTobeMapped);
    } else {
        Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
        increOdomAffine = increOdomAffine * affineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
        if (cloudInfo.imu_available == true && useImuRPYWeight)
        {
            if (std::abs(cloudInfo.imu_pitch_init) < 1.4)
            {
                double imuWeight = 0.1;
                tf2::Quaternion imuQuaternion;
                tf2::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(roll, 0, 0);
                imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                roll = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, pitch, 0);
                imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                pitch = pitchMid;
            }
        }
        laserOdomIncremental.header.stamp = timeLaserInfoStamp;
        laserOdomIncremental.header.frame_id = odometryFrame;
        laserOdomIncremental.child_frame_id = "odom_mapping";
        laserOdomIncremental.pose.pose.position.x = x;
        laserOdomIncremental.pose.pose.position.y = y;
        laserOdomIncremental.pose.pose.position.z = z;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdomIncremental.pose.pose.orientation = quat_msg;
        if (isDegenerate)
            laserOdomIncremental.pose.covariance[0] = 1;
        else
            laserOdomIncremental.pose.covariance[0] = 0;
    }
    pubLaserOdometryIncremental->publish(laserOdomIncremental);
}

void MapOptimization::publishFrames()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    // publish key poses
    publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
    // Publish surrounding key frames
    publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);

    // // publish registered key frame
    // if (pubRecentKeyFrame->get_subscription_count() != 0)
    // {
    //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    //     PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    //     *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
    //     *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
    //     publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
    // }
    // // publish registered high-res raw cloud
    // if (pubCloudRegisteredRaw->get_subscription_count() != 0)
    // {
    //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    //     pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
    //     PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    //     *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
    //     publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
    // }
    // publish path
    if (pubPath->get_subscription_count() != 0)
    {
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = odometryFrame;
        pubPath->publish(globalPath);
    }
}

void MapOptimization::updatePath(const PointTypePose& pose_in)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf2::Quaternion q;
    q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}



bool MapOptimization::saveMap(std::string destination, float resolution){
    struct stat info;

    string saveMapDirectory;
    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;
    if(destination.empty()) saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
    else saveMapDirectory = std::getenv("HOME") + destination;
    cout << "Save destination: " << saveMapDirectory << endl;

    // create directory and remove old files;
    if (stat(saveMapDirectory.c_str(), &info) == 0)
        system((std::string("exec rm -r ") + saveMapDirectory).c_str());
    system((std::string("mkdir -p ") + saveMapDirectory).c_str());


    std::function<bool()> saveDefault = [&]() -> bool {
        // save key frame transformations
        pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);

        // extract global point cloud map
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
        {
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
            *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }
        if(resolution != 0)
        {
            cout << "\n\nSave resolution: " << resolution << endl;
            // down-sample and save corner cloud
            downSizeFilterCorner.setInputCloud(globalCornerCloud);
            downSizeFilterCorner.setLeafSize(resolution, resolution, resolution);
            downSizeFilterCorner.filter(*globalCornerCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
            // down-sample and save surf cloud
            downSizeFilterSurf.setInputCloud(globalSurfCloud);
            downSizeFilterSurf.setLeafSize(resolution, resolution, resolution);
            downSizeFilterSurf.filter(*globalSurfCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
        }
        else
        {
            // save corner cloud
            pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
            // save surf cloud
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
        }
        // save global point cloud map
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;
        int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        return ret == 0;
    };

    std::function<bool()> saveKeyframeCloud = [&]() -> bool{
        // save key frame's point cloud
        string kfpcName = "keyframePointCloud/";
        if(saveMapDirectory.back() != '/')
            kfpcName = "/keyframePointCloud/";
        string keyframePointCloudDir = saveMapDirectory + kfpcName;
        if (stat(keyframePointCloudDir.c_str(), &info) == 0)
            system((std::string("exec rm -r ") + keyframePointCloudDir).c_str());
        system((std::string("mkdir -p ") + keyframePointCloudDir).c_str());


        bool ret = true;
        for (size_t i = 0; i < cornerCloudKeyFrames.size(); ++i) {
            std::stringstream ss;
            ss << keyframePointCloudDir << "corner_" << i << ".pcd";
            if(pcl::io::savePCDFile(ss.str(), *cornerCloudKeyFrames[i]) != 0)
                ret = false;
        }

        for (size_t i = 0; i < surfCloudKeyFrames.size(); ++i) {
            std::stringstream ss;
            ss << keyframePointCloudDir << "surf_"<< i << ".pcd";
            if(pcl::io::savePCDFile(ss.str(), *surfCloudKeyFrames[i]) != 0)
                ret = false;
        }

        return ret;
    };


    std::lock_guard<std::mutex> lock(mtx);
    bool ret = true;
    if(defaultMapInfo)
        ret = ret && saveDefault();
    if(keyframeCloud)
        ret = ret && saveKeyframeCloud();


    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;


    return ret;
}


