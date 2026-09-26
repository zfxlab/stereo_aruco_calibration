#ifndef STEREO_ARUCO_CALIBRATION_STABILITY_GATE_HPP
#define STEREO_ARUCO_CALIBRATION_STABILITY_GATE_HPP

#include <cstddef>
#include <opencv2/core/types.hpp>
#include <vector>

namespace stereo_calibration {

struct StabilityConfig {
    double minimum_duration_s{0.4};
    std::size_t minimum_pairs{6};
    double maximum_corner_motion_px{1.0};
    double rearm_corner_motion_px{8.0};
};

struct StabilityStatus {
    bool ready{};
    bool waiting_for_motion{};
    double progress{};
    std::size_t pair_count{};
    double duration_s{};
    double left_motion_px{};
    double right_motion_px{};
};

class StabilityGate {
public:
    explicit StabilityGate(StabilityConfig config);

    StabilityStatus update(double stamp_s, const std::vector<int>& marker_ids,
                           const std::vector<cv::Point2f>& left_points,
                           const std::vector<cv::Point2f>& right_points);
    void consume(const std::vector<int>& marker_ids,
                 const std::vector<cv::Point2f>& left_points,
                 const std::vector<cv::Point2f>& right_points);
    void reset();

private:
    void startWindow(double stamp_s, const std::vector<int>& marker_ids,
                     const std::vector<cv::Point2f>& left_points,
                     const std::vector<cv::Point2f>& right_points);

    StabilityConfig config_;
    bool waiting_for_motion_{};
    double window_start_s_{};
    std::size_t pair_count_{};
    std::vector<int> anchor_marker_ids_;
    std::vector<cv::Point2f> anchor_left_points_;
    std::vector<cv::Point2f> anchor_right_points_;
};

}  // namespace stereo_calibration
#endif
