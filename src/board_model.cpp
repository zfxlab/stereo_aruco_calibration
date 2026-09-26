#include "stereo_aruco_calibration/board_model.hpp"

#include <stdexcept>
#include <unordered_set>

namespace stereo_calibration {

BoardModel::BoardModel(const double marker_length_m,
                       const double center_spacing_x_m,
                       const double center_spacing_y_m,
                       const std::array<int, 4>& marker_ids)
    : marker_ids_(marker_ids) {
    if (!(marker_length_m > 0.0) || !(center_spacing_x_m > marker_length_m) ||
        !(center_spacing_y_m > marker_length_m)) {
        throw std::invalid_argument("Invalid marker length or center spacing");
    }
    const std::unordered_set<int> unique_ids(marker_ids.begin(), marker_ids.end());
    if (unique_ids.size() != marker_ids.size()) {
        throw std::invalid_argument("Board marker IDs must be unique");
    }

    const float half_marker = static_cast<float>(0.5 * marker_length_m);
    const float half_x = static_cast<float>(0.5 * center_spacing_x_m);
    const float half_y = static_cast<float>(0.5 * center_spacing_y_m);
    const std::array<cv::Point2f, 4> centers{{{-half_x, -half_y},
                                              {half_x, -half_y},
                                              {-half_x, half_y},
                                              {half_x, half_y}}};
    for (std::size_t index = 0; index < marker_ids.size(); ++index) {
        const auto center = centers[index];
        corners_.emplace(marker_ids[index],
                         std::array<cv::Point3f, 4>{{
                             {center.x - half_marker, center.y - half_marker, 0.0F},
                             {center.x + half_marker, center.y - half_marker, 0.0F},
                             {center.x + half_marker, center.y + half_marker, 0.0F},
                             {center.x - half_marker, center.y + half_marker, 0.0F},
                         }});
    }
}

std::optional<std::array<cv::Point3f, 4>> BoardModel::cornersForMarker(
    const int marker_id) const {
    const auto found = corners_.find(marker_id);
    if (found == corners_.end()) {
        return std::nullopt;
    }
    return found->second;
}

}  // namespace stereo_calibration

