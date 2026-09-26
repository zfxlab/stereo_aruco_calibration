#include "stereo_aruco_calibration/sample_manager.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

namespace stereo_calibration {
namespace {
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

double rotationDistanceDegrees(const cv::Vec3d& first, const cv::Vec3d& second) {
    cv::Mat first_rotation;
    cv::Mat second_rotation;
    cv::Rodrigues(first, first_rotation);
    cv::Rodrigues(second, second_rotation);
    const cv::Mat relative = first_rotation * second_rotation.t();
    const double cosine = std::clamp((cv::trace(relative)[0] - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine) * kRadiansToDegrees;
}
}  // namespace

SampleManager::SampleManager(SampleSelectionConfig config) : config_(config) {
    if (config_.minimum_points < 4 || config_.maximum_samples == 0 ||
        !(config_.minimum_rotation_deg >= 0.0) || !(config_.minimum_translation_m >= 0.0)) {
        throw std::invalid_argument("Invalid sample selection configuration");
    }
}

bool SampleManager::addIfDiverse(StereoSample sample, std::string& reason) {
    if (sample.object_points.size() < config_.minimum_points ||
        sample.object_points.size() != sample.left_points.size() ||
        sample.object_points.size() != sample.right_points.size()) {
        reason = "insufficient or inconsistent point correspondences";
        return false;
    }
    if (full()) {
        reason = "sample limit reached";
        return false;
    }
    for (const auto& existing : samples_) {
        const double rotation =
            rotationDistanceDegrees(sample.left_board_rvec, existing.left_board_rvec);
        const double translation = cv::norm(sample.left_board_tvec - existing.left_board_tvec);
        if (rotation < config_.minimum_rotation_deg &&
            translation < config_.minimum_translation_m) {
            reason = "pose is too similar to an existing sample";
            return false;
        }
    }
    samples_.push_back(std::move(sample));
    reason = "accepted";
    return true;
}

void SampleManager::clear() { samples_.clear(); }

}  // namespace stereo_calibration

