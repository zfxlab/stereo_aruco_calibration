#ifndef STEREO_ARUCO_CALIBRATION_SAMPLE_MANAGER_HPP
#define STEREO_ARUCO_CALIBRATION_SAMPLE_MANAGER_HPP
#include "stereo_aruco_calibration/calibration_types.hpp"
#include <string>
#include <vector>
namespace stereo_calibration {
struct SampleSelectionConfig {
    std::size_t minimum_points{12};
    std::size_t maximum_samples{40};
    double minimum_rotation_deg{5.0};
    double minimum_translation_m{0.08};
};
class SampleManager {
public:
    explicit SampleManager(SampleSelectionConfig config);
    bool addIfDiverse(StereoSample sample, std::string& reason);
    void clear();
    [[nodiscard]] std::vector<StereoSample> samples() const { return samples_; }
    [[nodiscard]] std::size_t size() const noexcept { return samples_.size(); }
    [[nodiscard]] bool full() const noexcept { return samples_.size() >= config_.maximum_samples; }
private:
    SampleSelectionConfig config_;
    std::vector<StereoSample> samples_;
};
}  // namespace stereo_calibration
#endif

