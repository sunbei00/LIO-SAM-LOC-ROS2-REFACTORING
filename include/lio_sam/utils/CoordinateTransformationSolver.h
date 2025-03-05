//
// Created by root on 25. 3. 2.
//

#ifndef COORDINATETRANSFORMATIONSOLVER_H
#define COORDINATETRANSFORMATIONSOLVER_H
#include "utils/pclType.h"

#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>


// 입력: R (최적화할 회전)
// 출력: 3차원 오차 벡터 (로그 맵 결과)
// argmin_R (relative_pose_1_inv * R * relative_pose_2)

class RotationFactor2D : public NoiseModelFactor1<Rot2> {
    Rot2 relative_pose_1_, relative_pose_2_;

    double normalizeAngle(double angle) const {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle <= -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }
public:
    RotationFactor2D(Key key, const Rot2& relative_pose_1, const Rot2& relative_pose_2,
                     const SharedNoiseModel& noiseModel)
            : NoiseModelFactor1<Rot2>(noiseModel, key),
              relative_pose_1_(relative_pose_1), relative_pose_2_(relative_pose_2) {}

    Vector evaluateError(const Rot2& R, boost::optional<Matrix&> H = boost::none) const override {
        double predicted_angle = R.theta() + relative_pose_2_.theta();
        double error_angle = normalizeAngle(predicted_angle - relative_pose_1_.theta());

        if (H) {
            *H = (Matrix(1,1) << 1.0).finished();
        }

//        RCLCPP_INFO(rclcpp::get_logger("localization"), "error: %f", error_angle);
        return (Vector(1) << error_angle).finished();
    }
};


class CoordinateTransformationSolver {
private:
    double angleDifference(double a, double b) {
        double diff = a - b;
        while (diff > M_PI)  diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        return diff;
    }
public:
  double solve(pcl::PointCloud<PointTypeXYI>::Ptr gpsKeyPoses2D,
             pcl::PointCloud<PointType>::Ptr keyframePose3D) {
      if (gpsKeyPoses2D->size() < 2) {
          std::cerr << "Not enough keyframes to compute closed-form rotation." << std::endl;
          return 0;
      }
    NonlinearFactorGraph graph;
    auto noise = noiseModel::Isotropic::Sigma(1, 3.0);


      for (size_t i = 0; i < gpsKeyPoses2D->size(); i++) {
          for (size_t j = i + 1; j < gpsKeyPoses2D->size(); j++) {
              int kfIdxi = (int)gpsKeyPoses2D->at(i).intensity;
              int kfIdxj = (int)gpsKeyPoses2D->at(j).intensity;

              double geographicYaw = atan2(gpsKeyPoses2D->at(j).y - gpsKeyPoses2D->at(i).y,
                                           gpsKeyPoses2D->at(j).x - gpsKeyPoses2D->at(i).x);
              double prebuiltMapYaw = atan2(keyframePose3D->at(kfIdxj).y - keyframePose3D->at(kfIdxi).y,
                                            keyframePose3D->at(kfIdxj).x - keyframePose3D->at(kfIdxi).x);

              Rot2 relative_pose_1 = Rot2::fromAngle(prebuiltMapYaw);
              Rot2 relative_pose_2 = Rot2::fromAngle(geographicYaw);

              //std::cout << "prebuiltYaw : " << prebuiltMapYaw << ", geographicYaw : " << geographicYaw << std::endl;

              graph.add(boost::make_shared<RotationFactor2D>(0, relative_pose_1, relative_pose_2, noise));
          }
      }
//     //      simple
//          for (size_t i = 1; i < gpsKeyPoses2D->size(); i++) {
//              int kfIdxi = (int)gpsKeyPoses2D->at(i).intensity; // keyframe index
//              int kfIdxim1 = (int)gpsKeyPoses2D->at(i-1).intensity; // keyframe index
//
//              double geographicYaw = atan2(gpsKeyPoses2D->at(i).y - gpsKeyPoses2D->at(i - 1).y,
//                                          gpsKeyPoses2D->at(i).x - gpsKeyPoses2D->at(i - 1).x);
//              double prebuiltMapYaw = atan2(keyframePose3D->at(kfIdxi).y - keyframePose3D->at(kfIdxim1).y,
//                                              keyframePose3D->at(kfIdxi).x - keyframePose3D->at(kfIdxim1).x);
//
//              Rot2 relative_pose_1 = Rot2::fromAngle(prebuiltMapYaw);
//              Rot2 relative_pose_2 = Rot2::fromAngle(geographicYaw);
//              //std::cout << "prebuiltYaw : " << prebuiltMapYaw << ", geographicYaw : " << geographicYaw << std::endl;
//
//              graph.add(boost::make_shared<RotationFactor2D>(0, relative_pose_1, relative_pose_2, noise));
//          }


        Values initial;
        initial.insert(0, Rot2::fromAngle(0.0));


        gtsam::LevenbergMarquardtParams params;
        params.setMaxIterations(1000);
        LevenbergMarquardtOptimizer optimizer(graph, initial, params);
          optimizer.optimize();
          optimizer.optimize();
          optimizer.optimize();
          optimizer.optimize();
        Values result = optimizer.optimize();

        Rot2 R_est = result.at<Rot2>(0);
        cout << "optimized R: " << R_est.theta() << " rad" << endl;

        return R_est.theta();
    }

    double solveClosedForm(pcl::PointCloud<PointTypeXYI>::Ptr gpsKeyPoses2D,
                         pcl::PointCloud<PointType>::Ptr keyframePose3D) {
        if (gpsKeyPoses2D->size() < 2) {
            std::cerr << "Not enough keyframes to compute closed-form rotation." << std::endl;
            return 0;
        }

        std::vector<double> deltaThetas;
//        for (size_t i = 1; i < gpsKeyPoses2D->size(); i++) {
//            int kfIdxi = static_cast<int>(gpsKeyPoses2D->at(i).intensity);
//            int kfIdxj = static_cast<int>(gpsKeyPoses2D->at(i - 1).intensity);
//
//            double geographicYaw = std::atan2(
//                    gpsKeyPoses2D->at(i).y - gpsKeyPoses2D->at(i - 1).y,
//                    gpsKeyPoses2D->at(i).x - gpsKeyPoses2D->at(i - 1).x);
//
//            double prebuiltMapYaw = std::atan2(
//                    keyframePose3D->at(kfIdxj).y - keyframePose3D->at(kfIdxi).y,
//                    keyframePose3D->at(kfIdxj).x - keyframePose3D->at(kfIdxi).x);
//
//            deltaThetas.push_back(angleDifference(prebuiltMapYaw, geographicYaw));
//        }

        for (size_t i = 0; i < gpsKeyPoses2D->size(); i++) {
            for (size_t j = i + 1; j < gpsKeyPoses2D->size(); j++) {
                int kfIdxi = (int) gpsKeyPoses2D->at(i).intensity;
                int kfIdxj = (int) gpsKeyPoses2D->at(j).intensity;

                double geographicYaw = atan2(gpsKeyPoses2D->at(j).y - gpsKeyPoses2D->at(i).y,
                                             gpsKeyPoses2D->at(j).x - gpsKeyPoses2D->at(i).x);

                double prebuiltMapYaw = std::atan2(
                        keyframePose3D->at(kfIdxj).y - keyframePose3D->at(kfIdxi).y,
                        keyframePose3D->at(kfIdxj).x - keyframePose3D->at(kfIdxi).x);

                deltaThetas.push_back(angleDifference(prebuiltMapYaw, geographicYaw));
            }
        }

        // 초기 평균 (초기 원형 평균 계산)
        double sumSin = 0.0, sumCos = 0.0;
        for (double d : deltaThetas) {
            sumSin += std::sin(d);
            sumCos += std::cos(d);
        }
        double initialMean = std::atan2(sumSin, sumCos);

        // 각 측정치와 초기 평균 간의 차이 계산
        std::vector<double> diffs;
        diffs.reserve(deltaThetas.size());
        for (double d : deltaThetas) {
            diffs.push_back(std::fabs(angleDifference(d, initialMean)));
        }

        // 중앙값(median) 계산
        size_t medianIdx = diffs.size() / 2;
        std::nth_element(diffs.begin(), diffs.begin() + medianIdx, diffs.end());
        double medianDiff = diffs[medianIdx];

        // Threshold = 중앙값의 2배 (이상치 제거)
        double threshold = 2.0 * medianDiff;

        // inlier 평균 계산 (이상치 필터링)
        double inlierSumSin = 0.0, inlierSumCos = 0.0;
        int inlierCount = 0;
        for (double d : deltaThetas) {
            if (std::fabs(angleDifference(d, initialMean)) <= threshold) {
                inlierSumSin += std::sin(d);
                inlierSumCos += std::cos(d);
                inlierCount++;
            }
        }

        // inlier가 없는 경우 초기 평균 사용
        double finalRotation = (inlierCount > 0) ? std::atan2(inlierSumSin, inlierSumCos) : initialMean;

        Rot2 R_est = Rot2::fromAngle(finalRotation);
        std::cout << "Closed-form optimized R (with outlier removal): " << R_est.theta()
                  << " rad, inliers: " << inlierCount << "/" << deltaThetas.size() << std::endl;

        return R_est.theta();
    }
};



#endif //COORDINATETRANSFORMATIONSOLVER_H
