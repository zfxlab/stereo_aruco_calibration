#ifndef STEREO_ARUCO_CALIBRATION_CALIBRATION_TYPES_HPP
#define STEREO_ARUCO_CALIBRATION_CALIBRATION_TYPES_HPP
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace stereo_calibration {
struct CameraIntrinsics {
    cv::Mat camera_matrix;
    cv::Mat distortion;
    cv::Size image_size;
    std::string frame_id;
    [[nodiscard]] bool valid() const noexcept;
};
struct StereoSample {
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    cv::Vec3d left_board_rvec{};
    cv::Vec3d left_board_tvec{};
    double stamp_delta_s{};
};
struct StereoCalibrationResult {
    cv::Mat rotation_right_left;
    cv::Mat translation_right_left;
    cv::Mat essential;
    cv::Mat fundamental;
    double stereo_rms_px{};
    double mean_epipolar_error_px{};
    double max_epipolar_error_px{};
    double baseline_m{};
    std::size_t sample_count{};
    std::size_t point_count{};
};
}  // namespace stereo_calibration
#endif

