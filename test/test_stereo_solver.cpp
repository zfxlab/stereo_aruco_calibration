#include "stereo_aruco_calibration/board_model.hpp"
#include "stereo_aruco_calibration/stereo_solver.hpp"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

TEST(StereoSolver, RecoversKnownRelativeTransform) {
    stereo_calibration::CameraIntrinsics left;
    left.camera_matrix = (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 640.0,
                                                      0.0, 1000.0, 480.0,
                                                      0.0, 0.0, 1.0);
    left.distortion = cv::Mat::zeros(1, 5, CV_64F);
    left.image_size = {1280, 960};
    left.frame_id = "left_optical";
    auto right = left;
    right.frame_id = "right_optical";

    const stereo_calibration::BoardModel board(0.20, 1.10, 0.70, {1, 2, 3, 4});
    std::vector<cv::Point3f> object_points;
    for (const int id : board.markerIds()) {
        const auto corners = board.cornersForMarker(id).value();
        object_points.insert(object_points.end(), corners.begin(), corners.end());
    }

    const cv::Vec3d relative_rvec(0.002, -0.004, 0.003);
    cv::Mat relative_rotation;
    cv::Rodrigues(relative_rvec, relative_rotation);
    const cv::Mat relative_translation = (cv::Mat_<double>(3, 1) << -0.30, 0.002, 0.001);
    std::vector<stereo_calibration::StereoSample> samples;
    for (int index = 0; index < 12; ++index) {
        stereo_calibration::StereoSample sample;
        sample.object_points = object_points;
        sample.left_board_rvec = cv::Vec3d(0.03 * (index % 3),
                                            -0.04 * ((index / 3) % 3),
                                            0.02 * index);
        sample.left_board_tvec = cv::Vec3d(-0.15 + 0.03 * index,
                                            -0.08 + 0.02 * (index % 5),
                                            4.0 + 0.15 * index);
        cv::projectPoints(object_points, sample.left_board_rvec, sample.left_board_tvec,
                          left.camera_matrix, left.distortion, sample.left_points);
        cv::Mat left_rotation;
        cv::Rodrigues(sample.left_board_rvec, left_rotation);
        const cv::Mat right_board_rotation = relative_rotation * left_rotation;
        const cv::Mat left_translation = (cv::Mat_<double>(3, 1) <<
            sample.left_board_tvec[0], sample.left_board_tvec[1], sample.left_board_tvec[2]);
        const cv::Mat right_board_translation =
            relative_rotation * left_translation + relative_translation;
        cv::Vec3d right_board_rvec;
        cv::Rodrigues(right_board_rotation, right_board_rvec);
        cv::projectPoints(object_points, right_board_rvec, right_board_translation,
                          right.camera_matrix, right.distortion, sample.right_points);
        samples.push_back(std::move(sample));
    }

    stereo_calibration::StereoCalibrationResult result;
    std::string error;
    ASSERT_TRUE(stereo_calibration::StereoSolver::solve(samples, left, right, result, error))
        << error;
    EXPECT_LT(cv::norm(result.rotation_right_left - relative_rotation), 1e-4);
    EXPECT_LT(cv::norm(result.translation_right_left - relative_translation), 1e-4);
    EXPECT_NEAR(result.baseline_m, cv::norm(relative_translation), 1e-4);
    EXPECT_LT(result.stereo_rms_px, 1e-3);
}
