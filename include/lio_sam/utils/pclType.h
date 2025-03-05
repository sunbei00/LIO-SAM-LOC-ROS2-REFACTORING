//
// Created by root on 11/2/24.
//

#ifndef BUILD_PCLTYPE_H
#define BUILD_PCLTYPE_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>


typedef pcl::PointXYZI PointType;

// it is come from imageProjection
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
            PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
(uint16_t, ring, ring) (float, time, time)
)

// it is come from imageProjection
struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
(uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
(uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
// it is come from mapOptimization
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
            PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                    (float, x, x) (float, y, y)
                                    (float, z, z) (float, intensity, intensity)
                                    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                    (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

struct PointXYI
{
    double x;
    double y;
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYI,
                                  (double, x, x)
                                  (double, y, y)
                                  (float, intensity, intensity))

typedef PointXYI PointTypeXYI;


#endif //BUILD_PCLTYPE_H
