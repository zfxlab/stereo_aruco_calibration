#ifndef STEREO_ARUCO_CALIBRATION_BOARD_MODEL_HPP
#define STEREO_ARUCO_CALIBRATION_BOARD_MODEL_HPP
#include <array>
#include <opencv2/core.hpp>
#include <optional>
#include <unordered_map>
namespace stereo_calibration {
class BoardModel {
public:
    BoardModel(double marker_length_m, double center_spacing_x_m,
               double center_spacing_y_m, const std::array<int, 4>& marker_ids);
    [[nodiscard]] std::optional<std::array<cv::Point3f, 4>>
    cornersForMarker(int marker_id) const;
    [[nodiscard]] const std::array<int, 4>& markerIds() const noexcept { return marker_ids_; }
private:
    std::array<int, 4> marker_ids_{};
    std::unordered_map<int, std::array<cv::Point3f, 4>> corners_;
};
}  // namespace stereo_calibration
#endif

