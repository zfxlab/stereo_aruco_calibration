#ifndef STEREO_ARUCO_CALIBRATION_CALIBRATOR_NODE_HPP
#define STEREO_ARUCO_CALIBRATION_CALIBRATOR_NODE_HPP

#include "stereo_aruco_calibration/board_model.hpp"
#include "stereo_aruco_calibration/calibration_types.hpp"
#include "stereo_aruco_calibration/sample_manager.hpp"
#include "stereo_aruco_calibration/stability_gate.hpp"

#include <array>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <mutex>
#include <opencv2/aruco.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unordered_map>

namespace stereo_calibration {

class StereoArucoCalibratorNode : public rclcpp::Node {
public:
    explicit StereoArucoCalibratorNode(const rclcpp::NodeOptions& options);

private:
    struct Observation {
        rclcpp::Time stamp;
        std::string frame_id;
        cv::Size image_size;
        std::unordered_map<int, std::array<cv::Point2f, 4>> markers;
        cv::Mat preview;
    };

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message,
                            bool is_left);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& message, bool is_left);
    std::optional<Observation> detect(const sensor_msgs::msg::Image::ConstSharedPtr& message);
    void queueObservation(Observation observation, bool is_left);
    void matchObservations();
    void processPair(const Observation& left, const Observation& right);
    void publishLatestPreview();
    void publishPreview(const Observation& left, const Observation& right,
                        const std::vector<int>& common_ids, bool capturing);
    void publishStatus();
    void setDecision(const std::string& decision);

    CameraIntrinsics left_intrinsics_;
    CameraIntrinsics right_intrinsics_;
    std::unique_ptr<BoardModel> board_;
    std::unique_ptr<SampleManager> sample_manager_;
    std::unique_ptr<StabilityGate> stability_gate_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

    std::deque<Observation> left_queue_;
    std::deque<Observation> right_queue_;
    std::optional<Observation> latest_left_observation_;
    std::optional<Observation> latest_right_observation_;
    std::mutex mutex_;
    bool capturing_{};
    bool solving_{};
    std::optional<StereoCalibrationResult> last_result_;
    rclcpp::Time last_left_processed_;
    rclcpp::Time last_right_processed_;

    double max_pair_delta_s_{};
    double minimum_process_interval_s_{};
    double maximum_reprojection_error_px_{};
    double stability_minimum_duration_s_{};
    double stability_maximum_corner_motion_px_{};
    double stability_rearm_corner_motion_px_{};
    double preview_interval_s_{};
    double preview_scale_{};
    int jpeg_quality_{};
    int minimum_common_markers_{};
    int minimum_samples_{};
    int maximum_samples_{};
    std::size_t queue_size_{};
    std::size_t stability_minimum_pairs_{};
    bool preview_enabled_{};
    std::string output_path_;

    std::string last_decision_{"waiting_for_images"};
    std::size_t last_left_marker_count_{};
    std::size_t last_right_marker_count_{};
    std::size_t last_common_marker_count_{};
    double last_pair_delta_ms_{std::numeric_limits<double>::quiet_NaN()};
    double last_left_reprojection_px_{std::numeric_limits<double>::quiet_NaN()};
    double last_right_reprojection_px_{std::numeric_limits<double>::quiet_NaN()};
    std::string sampling_state_{"WAITING_FOR_MARKERS"};
    double stability_progress_{};
    std::size_t stability_pair_count_{};
    double stability_duration_ms_{};
    double last_left_corner_motion_px_{std::numeric_limits<double>::quiet_NaN()};
    double last_right_corner_motion_px_{std::numeric_limits<double>::quiet_NaN()};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr preview_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr solve_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};

}  // namespace stereo_calibration
#endif
