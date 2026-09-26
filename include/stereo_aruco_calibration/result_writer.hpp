#ifndef STEREO_ARUCO_CALIBRATION_RESULT_WRITER_HPP
#define STEREO_ARUCO_CALIBRATION_RESULT_WRITER_HPP
#include "stereo_aruco_calibration/calibration_types.hpp"
#include <string>
namespace stereo_calibration {
class ResultWriter {
public:
    static bool write(const std::string& path, const CameraIntrinsics& left,
                      const CameraIntrinsics& right,
                      const StereoCalibrationResult& result, std::string& error);
};
}  // namespace stereo_calibration
#endif
