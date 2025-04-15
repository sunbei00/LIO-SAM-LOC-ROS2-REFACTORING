#include "MapOptimization.h"
#include "utils/gtsamUtils.h"
#include "utils/gpsUtils.h"
#include <opencv2/opencv.hpp>



void MapOptimization::navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
    double lat = msg->latitude;
    double lon = msg->longitude;
    auto [east, north] = latlon_to_utm(lat, lon);

    currGPS.x = east;
    currGPS.y = north;
    currGPS.intensity = 0;
}

void MapOptimization::gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg)
{
    gpsQueue.push_back(*gpsMsg);
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}


void MapOptimization::laserCloudInfoHandler(const lio_sam_loc::msg::CloudInfo::SharedPtr msgIn)
{
    // extract time stamp
    timeLaserInfoStamp = msgIn->header.stamp;
    timeLaserInfoCur = stamp2Sec(msgIn->header.stamp);

    // extract info and feature cloud
    cloudInfo = *msgIn;
    pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
    pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

    std::lock_guard<std::mutex> lock(mtx);

    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
    {
        timeLastProcessing = timeLaserInfoCur;

        downsampleCurrentScan();

        // localization -------------------------------------------
        if (!system_initialized)
            if(!systemInitialize())
                return;

        // --------------------------------------------------------

        updateInitialGuess();

        extractSurroundingKeyFrames();

        scan2MapOptimization();

        saveKeyFramesAndFactor();

        correctPoses();

        publishOdometry();

        publishFrames();
    }
}

void MapOptimization::updateInitialGuess()
{
    // save current transformation before any processing
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    static Eigen::Affine3f lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, imuType==9 ? cloudInfo.imu_yaw_init : 0);
    // use imu pre-integration estimation for pose guess
    static bool lastImuPreTransAvailable = false;

    static Eigen::Affine3f lastImuPreTransformation;
    if (cloudInfo.odom_available == true)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(
            cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,
            cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
        if (lastImuPreTransAvailable == false)
        {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        } else {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuPreTransformation = transBack;

            if (imuType == 8 || imuType == 9)
                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, imuType==9 ? cloudInfo.imu_yaw_init : 0);
            return;
        }
    }

    // use imu incremental estimation for pose guess (only rotation)
    if (cloudInfo.imu_available == true && (imuType == 8 || imuType == 9))
    {
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, imuType==9 ? cloudInfo.imu_yaw_init : 0);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                      transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, imuType==9 ? cloudInfo.imu_yaw_init : 0); // save imu before return;
        return;
    }
}


void MapOptimization::extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());

    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses - 1; i >= std::max(0, numPoses - numberOfKeyframeForTempolarMap); --i)
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[i]);

    extractCloud(surroundingKeyPoses);
}

void MapOptimization::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
        {
            // transformed cloud available
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        } else {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);

            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap   += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }

    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);

    // clear map cache if too large
    if (laserCloudMapContainer.size() > 50)
        laserCloudMapContainer.clear();
}

void MapOptimization::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    extractNearby();
}

void MapOptimization::downsampleCurrentScan()
{
    // Downsample cloud from current scan
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

void MapOptimization::cornerOptimization()
{
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudCornerLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudCornerLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel, trans2Affine3f(transformTobeMapped));
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

        if (pointSearchSqDis[4] < 1.0) {
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
            }
            cx /= 5; cy /= 5;  cz /= 5;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
                float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                a22 += ay * ay; a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

            matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

            cv::eigen(matA1, matD1, matV1);

            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                          + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                           + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1 - 0.9 * fabs(ld2);

                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                coeff.intensity = s * ld2;

                if (s > 0.1) {
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    laserCloudOriCornerFlag[i] = true;
                }
            }
        }
    }
}

void MapOptimization::surfOptimization()
{

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel, trans2Affine3f(transformTobeMapped));
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                         pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                         pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                        + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

void MapOptimization::combineOptimizationCoeffs()
{
    // combine corner coeffs
    for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
        if (laserCloudOriCornerFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriCornerVec[i]);
            coeffSel->push_back(coeffSelCornerVec[i]);
        }
    }
    // combine surf coeffs
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    // reset flag for next iteration
    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool MapOptimization::LMOptimization(int iterCount)
{
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(transformTobeMapped[1]);
    float crx = cos(transformTobeMapped[1]);
    float sry = sin(transformTobeMapped[2]);
    float cry = cos(transformTobeMapped[2]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++) {
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].y;
        pointOri.y = laserCloudOri->points[i].z;
        pointOri.z = laserCloudOri->points[i].x;
        // lidar -> camera
        coeff.x = coeffSel->points[i].y;
        coeff.y = coeffSel->points[i].z;
        coeff.z = coeffSel->points[i].x;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                  + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                  + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                  + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                  + ((-cry*crz - srx*sry*srz)*pointOri.x
                  + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                  + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                  + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
        // lidar -> camera
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {

        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        return true; // converged
    }
    return false; // keep optimizing
}

void MapOptimization::scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            laserCloudOri->clear();
            coeffSel->clear();

            cornerOptimization();
            surfOptimization();

            combineOptimizationCoeffs();

            if (LMOptimization(iterCount) == true)
                break;
        }

        transformUpdate();
    } else {
        RCLCPP_WARN(get_logger(), "Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
    }
}


void MapOptimization::transformUpdate()
{
    if (cloudInfo.imu_available == true)
    {
        if (std::abs(cloudInfo.imu_pitch_init) < 1.4 && useImuRPYWeight) // // useImuHeadingInitialization - true : 9Axis-IMU, false : 6Axis-IMU
        {
            double imuWeight = imuRPYWeight;
            tf2::Quaternion imuQuaternion;
            tf2::Quaternion transformQuaternion;
            double rollMid, pitchMid, yawMid;

            // slerp roll
            transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
            imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
            tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[0] = rollMid;

            // slerp pitch
            transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
            imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
            tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[1] = pitchMid;
        }
    }

    // constraintTransformation
    transformTobeMapped[0] = std::clamp(transformTobeMapped[0], -rotation_tollerance, rotation_tollerance);
    transformTobeMapped[1] = std::clamp(transformTobeMapped[1], -rotation_tollerance, rotation_tollerance);
    transformTobeMapped[5] = std::clamp(transformTobeMapped[5], -z_tollerance, z_tollerance);

    incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
}


bool MapOptimization::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    if (sensor == SensorType::LIVOX)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
            return true;
    }

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;
}

void MapOptimization::addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }else{
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

void MapOptimization::addGPSFactor()
{
    if (gpsQueue.empty())
        return;

    // wait for system initialized and settles down
    if (cloudKeyPoses3D->points.empty())
        return;
    else
    {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }

    // pose covariance small, no need to correct
    if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        return;

    // last gps position
    static PointType lastGPSPoint;

    while (!gpsQueue.empty())
    {
        if (stamp2Sec(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2)
        {
            // message too old
            gpsQueue.pop_front();
        }
        else if (stamp2Sec(gpsQueue.front().header.stamp) > timeLaserInfoCur + 0.2)
        {
            // message too new
            break;
        }
        else
        {
            nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
            gpsQueue.pop_front();

            // GPS too noisy, skip
            float noise_x = thisGPS.pose.covariance[0];
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];
            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                continue;
            float gps_x = thisGPS.pose.pose.position.x;
            float gps_y = thisGPS.pose.pose.position.y;
            float gps_z = thisGPS.pose.pose.position.z;
            if (!useGpsElevation)
            {
                gps_z = transformTobeMapped[5];
                noise_z = 0.01;
            }

            // GPS not properly initialized (0,0,0)
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                continue;

            // Add GPS every a few meters
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                continue;
            else
                lastGPSPoint = curGPSPoint;

            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);

            aLoopIsClosed = true;
            break;
        }
    }
}

void MapOptimization::addGlobalMatchingFactor() {
    if(globalMatchingResult.empty())
        return;

    mtxGlobalMatching.lock();
    auto [idx, roll, pitch, yaw, x, y, z, rot_var, pos_var] = globalMatchingResult.back();
    globalMatchingResult.pop_back();
    mtxGlobalMatching.unlock();

    gtsam::noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Variances(
            (Vector(6) << pos_var, pos_var, pos_var, pos_var, pos_var, pos_var).finished());
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(
            idx,
            gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z)),
            pose_noise);
    gtSAMgraph.add(pose_factor);

    aLoopIsClosed = true;
}

void MapOptimization::saveKeyFramesAndFactor()
{
    if (saveFrame() == false)
        return;

    // odom factor
    addOdomFactor();

    // gps factor
    addGPSFactor();

    // global globalMatching factor
    addGlobalMatchingFactor();

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index

    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll  = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw   = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

    // save key frame cloud
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    // save path for visualization
    updatePath(thisPose6D);
}


void MapOptimization::correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // clear map cache
        laserCloudMapContainer.clear();
        // clear path
        globalPath.poses.clear();
        // update key poses
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

            updatePath(cloudKeyPoses6D->points[i]);
        }

        aLoopIsClosed = false;
    }
}

