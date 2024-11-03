//
// Created by root on 11/2/24.
//

#ifndef BUILD_FEATUREEXTRACTION_H
#define BUILD_FEATUREEXTRACTION_H

#include "lio_sam_loc/msg/cloud_info.hpp"

#include "utils/ParamServer.h"
#include "utils/pclType.h"

#include <pcl/filters/voxel_grid.h>


struct smoothness_t{
    float value;
    size_t ind;
};

struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer{
public:
    rclcpp::Subscription<lio_sam_loc::msg::CloudInfo>::SharedPtr subLaserCloudInfo;

    rclcpp::Publisher<lio_sam_loc::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;                     // deskew point cloud
    pcl::PointCloud<PointType>::Ptr cornerCloud;                        // corner point cloud
    pcl::PointCloud<PointType>::Ptr surfaceCloud;                       // surf point cloud

    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam_loc::msg::CloudInfo cloudInfo;
    std_msgs::msg::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    explicit FeatureExtraction(const rclcpp::NodeOptions & options);
    void initializationValue();
    void laserCloudInfoHandler(lio_sam_loc::msg::CloudInfo::SharedPtr msgIn);
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();
    void freeCloudInfoMemory();
    void publishFeatureCloud();
};




#endif //BUILD_FEATUREEXTRACTION_H
