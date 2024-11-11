//
// Created by root on 11/2/24.
//

#ifndef BUILD_GTSAMUTILS_H
#define BUILD_GTSAMUTILS_H
#include "utils/pclType.h"
#include <gtsam/geometry/Pose3.h>


gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
gtsam::Pose3 trans2gtsamPose(float transformIn[]);


#endif //BUILD_GTSAMUTILS_H
