#include "stereo_aruco_calibration/stereo_solver.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <sstream>

namespace stereo_calibration {

bool CameraIntrinsics::valid() const noexcept {
    return camera_matrix.rows == 3 && camera_matrix.cols == 3 &&
           camera_matrix.type() == CV_64F && image_size.width > 0 && image_size.height > 0 &&
           std::isfinite(camera_matrix.at<double>(0, 0)) &&
           std::isfinite(camera_matrix.at<double>(1, 1)) &&
           camera_matrix.at<double>(0, 0) > 0.0 && camera_matrix.at<double>(1, 1) > 0.0 &&
           !frame_id.empty();
}

bool StereoSolver::solve(const std::vector<StereoSample>& samples,
                         const CameraIntrinsics& left,
                         const CameraIntrinsics& right,
                         StereoCalibrationResult& result,
                         std::string& error) {
    if (!left.valid() || !right.valid()) {
        error = "left or right intrinsics are invalid";
        return false;
    }
    if (left.image_size != right.image_size) {
        error = "left and right image sizes differ";
        return false;
    }
    if (samples.size() < 2) {
        error = "at least two diverse samples are required";
        return false;
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> left_points;
    std::vector<std::vector<cv::Point2f>> right_points;
    std::size_t point_count = 0;
    for (const auto& sample : samples) {
        if (sample.object_points.size() < 4 ||
            sample.object_points.size() != sample.left_points.size() ||
            sample.object_points.size() != sample.right_points.size()) {
            error = "a sample contains inconsistent point counts";
            return false;
        }
        object_points.push_back(sample.object_points);
        left_points.push_back(sample.left_points);
        right_points.push_back(sample.right_points);
        point_count += sample.object_points.size();
    }

    cv::Mat left_matrix = left.camera_matrix.clone();
    cv::Mat right_matrix = right.camera_matrix.clone();
    cv::Mat left_distortion = left.distortion.clone();
    cv::Mat right_distortion = right.distortion.clone();
    cv::Mat rotation;
    cv::Mat translation;
    cv::Mat essential;
    cv::Mat fundamental;
    try {
        const double rms = cv::stereoCalibrate(
            object_points, left_points, right_points,
            left_matrix, left_distortion, right_matrix, right_distortion,
            left.image_size, rotation, translation, essential, fundamental,
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-10));

        double total_error = 0.0;
        double maximum_error = 0.0;
        std::size_t error_count = 0;
        for (std::size_t view = 0; view < left_points.size(); ++view) {
            std::vector<cv::Vec3f> right_lines;
            std::vector<cv::Vec3f> left_lines;
            cv::computeCorrespondEpilines(left_points[view], 1, fundamental, right_lines);
            cv::computeCorrespondEpilines(right_points[view], 2, fundamental, left_lines);
            for (std::size_t index = 0; index < left_points[view].size(); ++index) {
                const auto pointLineDistance = [](const cv::Point2f& point,
                                                  const cv::Vec3f& line) {
                    const double denominator = std::hypot(line[0], line[1]);
                    return denominator > 0.0
                               ? std::abs(line[0] * point.x + line[1] * point.y + line[2]) /
                                     denominator
                               : std::numeric_limits<double>::infinity();
                };
                const double left_error = pointLineDistance(left_points[view][index], left_lines[index]);
                const double right_error = pointLineDistance(right_points[view][index], right_lines[index]);
                const double symmetric_error = 0.5 * (left_error + right_error);
                total_error += symmetric_error;
                maximum_error = std::max(maximum_error, symmetric_error);
                ++error_count;
            }
        }

        result.rotation_right_left = rotation.clone();
        result.translation_right_left = translation.clone();
        result.essential = essential.clone();
        result.fundamental = fundamental.clone();
        result.stereo_rms_px = rms;
        result.mean_epipolar_error_px = total_error / static_cast<double>(error_count);
        result.max_epipolar_error_px = maximum_error;
        result.baseline_m = cv::norm(translation);
        result.sample_count = samples.size();
        result.point_count = point_count;
    } catch (const cv::Exception& exception) {
        error = std::string("OpenCV stereo calibration failed: ") + exception.what();
        return false;
    }
    if (!std::isfinite(result.stereo_rms_px) || !std::isfinite(result.baseline_m) ||
        result.baseline_m <= 0.0) {
        error = "solver returned non-finite or degenerate calibration";
        return false;
    }
    error.clear();
    return true;
}

}  // namespace stereo_calibration

