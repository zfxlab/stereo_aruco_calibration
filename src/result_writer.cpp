#include "stereo_aruco_calibration/result_writer.hpp"

#include <opencv2/core.hpp>

namespace stereo_calibration {

bool ResultWriter::write(const std::string& path,
                         const CameraIntrinsics& left,
                         const CameraIntrinsics& right,
                         const StereoCalibrationResult& result,
                         std::string& error) {
    try {
        cv::FileStorage storage(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
        if (!storage.isOpened()) {
            error = "cannot open output path: " + path;
            return false;
        }
        storage << "format_version" << 1;
        storage << "transform_convention"
                << "X_right = R_right_left * X_left + t_right_left";
        storage << "length_unit" << "meter";
        storage << "left_frame_id" << left.frame_id;
        storage << "right_frame_id" << right.frame_id;
        storage << "image_width" << left.image_size.width;
        storage << "image_height" << left.image_size.height;
        storage << "rotation_right_left" << result.rotation_right_left;
        storage << "translation_right_left_m" << result.translation_right_left;
        storage << "essential_matrix" << result.essential;
        storage << "fundamental_matrix" << result.fundamental;
        storage << "metrics" << "{";
        storage << "sample_count" << static_cast<int>(result.sample_count);
        storage << "point_count" << static_cast<int>(result.point_count);
        storage << "stereo_rms_px" << result.stereo_rms_px;
        storage << "mean_epipolar_error_px" << result.mean_epipolar_error_px;
        storage << "max_epipolar_error_px" << result.max_epipolar_error_px;
        storage << "baseline_m" << result.baseline_m;
        storage << "}";
        storage.release();
    } catch (const cv::Exception& exception) {
        error = std::string("cannot write calibration result: ") + exception.what();
        return false;
    }
    error.clear();
    return true;
}

}  // namespace stereo_calibration
