#ifndef STEREO_ARUCO_CALIBRATION_STEREO_SOLVER_HPP
#define STEREO_ARUCO_CALIBRATION_STEREO_SOLVER_HPP
#include "stereo_aruco_calibration/calibration_types.hpp"
#include <string>
#include <vector>
namespace stereo_calibration {
class StereoSolver {
public:
    static bool solve(const std::vector<StereoSample>& samples,
                      const CameraIntrinsics& left, const CameraIntrinsics& right,
                      StereoCalibrationResult& result, std::string& error);
};
}  // namespace stereo_calibration
#endif

