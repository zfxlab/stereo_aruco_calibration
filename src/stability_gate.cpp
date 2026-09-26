#include "stereo_aruco_calibration/stability_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace stereo_calibration {
namespace {

double cornerRms(const std::vector<cv::Point2f>& first,
                 const std::vector<cv::Point2f>& second) {
    if (first.size() != second.size() || first.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double squared_sum = 0.0;
    for (std::size_t index = 0; index < first.size(); ++index) {
        const auto delta = first[index] - second[index];
        squared_sum += static_cast<double>(delta.dot(delta));
    }
    return std::sqrt(squared_sum / static_cast<double>(first.size()));
}

}  // namespace

StabilityGate::StabilityGate(StabilityConfig config) : config_(config) {
    if (!(config_.minimum_duration_s >= 0.0) || config_.minimum_pairs == 0 ||
        !(config_.maximum_corner_motion_px >= 0.0) ||
        !(config_.rearm_corner_motion_px > config_.maximum_corner_motion_px)) {
        throw std::invalid_argument("Invalid stability configuration");
    }
}

void StabilityGate::startWindow(double stamp_s, const std::vector<int>& marker_ids,
                                const std::vector<cv::Point2f>& left_points,
                                const std::vector<cv::Point2f>& right_points) {
    window_start_s_ = stamp_s;
    pair_count_ = 1;
    anchor_marker_ids_ = marker_ids;
    anchor_left_points_ = left_points;
    anchor_right_points_ = right_points;
}

StabilityStatus StabilityGate::update(double stamp_s, const std::vector<int>& marker_ids,
                                      const std::vector<cv::Point2f>& left_points,
                                      const std::vector<cv::Point2f>& right_points) {
    if (marker_ids.empty() || left_points.size() != right_points.size() ||
        left_points.size() != marker_ids.size() * 4U) {
        throw std::invalid_argument("Stability observations are inconsistent");
    }

    if (anchor_marker_ids_.empty()) {
        startWindow(stamp_s, marker_ids, left_points, right_points);
        return {false, false, 0.0, pair_count_, 0.0, 0.0, 0.0};
    }

    const bool comparable = marker_ids == anchor_marker_ids_;
    const double left_motion = comparable ? cornerRms(anchor_left_points_, left_points)
                                          : std::numeric_limits<double>::infinity();
    const double right_motion = comparable ? cornerRms(anchor_right_points_, right_points)
                                           : std::numeric_limits<double>::infinity();

    if (waiting_for_motion_) {
        if (!comparable || std::max(left_motion, right_motion) >= config_.rearm_corner_motion_px) {
            waiting_for_motion_ = false;
            startWindow(stamp_s, marker_ids, left_points, right_points);
            return {false, false, 0.0, pair_count_, 0.0, 0.0, 0.0};
        }
        return {false, true, 0.0, 0, 0.0, left_motion, right_motion};
    }

    if (!comparable || stamp_s < window_start_s_ ||
        std::max(left_motion, right_motion) > config_.maximum_corner_motion_px) {
        startWindow(stamp_s, marker_ids, left_points, right_points);
        return {false, false, 0.0, pair_count_, 0.0, 0.0, 0.0};
    }

    ++pair_count_;
    const double duration = stamp_s - window_start_s_;
    const double duration_progress = config_.minimum_duration_s == 0.0
                                         ? 1.0
                                         : duration / config_.minimum_duration_s;
    const double pair_progress = static_cast<double>(pair_count_) /
                                 static_cast<double>(config_.minimum_pairs);
    const double progress = std::clamp(std::min(duration_progress, pair_progress), 0.0, 1.0);
    const bool ready = duration >= config_.minimum_duration_s &&
                       pair_count_ >= config_.minimum_pairs;
    return {ready, false, progress, pair_count_, duration, left_motion, right_motion};
}

void StabilityGate::consume(const std::vector<int>& marker_ids,
                            const std::vector<cv::Point2f>& left_points,
                            const std::vector<cv::Point2f>& right_points) {
    waiting_for_motion_ = true;
    pair_count_ = 0;
    anchor_marker_ids_ = marker_ids;
    anchor_left_points_ = left_points;
    anchor_right_points_ = right_points;
}

void StabilityGate::reset() {
    waiting_for_motion_ = false;
    window_start_s_ = 0.0;
    pair_count_ = 0;
    anchor_marker_ids_.clear();
    anchor_left_points_.clear();
    anchor_right_points_.clear();
}

}  // namespace stereo_calibration
