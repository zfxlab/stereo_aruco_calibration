#include "stereo_aruco_calibration/calibrator_node.hpp"

#include "stereo_aruco_calibration/result_writer.hpp"
#include "stereo_aruco_calibration/stereo_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <functional>
#include <iomanip>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace stereo_calibration {
namespace {

CameraIntrinsics intrinsicsFrom(const sensor_msgs::msg::CameraInfo& message) {
    CameraIntrinsics result;
    result.camera_matrix =
        cv::Mat(3, 3, CV_64F, const_cast<double*>(message.k.data())).clone();
    if (!message.d.empty()) {
        result.distortion = cv::Mat(message.d, true).reshape(1, 1);
        result.distortion.convertTo(result.distortion, CV_64F);
    }
    result.image_size = {static_cast<int>(message.width), static_cast<int>(message.height)};
    result.frame_id = message.header.frame_id;
    return result;
}

double meanReprojectionError(const std::vector<cv::Point3f>& object_points,
                             const std::vector<cv::Point2f>& image_points,
                             const cv::Vec3d& rvec,
                             const cv::Vec3d& tvec,
                             const CameraIntrinsics& intrinsics) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, intrinsics.camera_matrix,
                      intrinsics.distortion, projected);
    double total = 0.0;
    for (std::size_t index = 0; index < projected.size(); ++index) {
        total += cv::norm(projected[index] - image_points[index]);
    }
    return projected.empty() ? std::numeric_limits<double>::infinity()
                             : total / static_cast<double>(projected.size());
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
}

std::string number(const double value, const int precision = 3) {
    if (!std::isfinite(value)) {
        return "nan";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

double clamp01(const double value) { return std::clamp(value, 0.0, 1.0); }

struct Coverage {
    double position{};
    double distance{};
    double rotation{};
};

Coverage calculateCoverage(const std::vector<StereoSample>& samples) {
    if (samples.size() < 2) {
        return {};
    }
    double min_horizontal = std::numeric_limits<double>::infinity();
    double max_horizontal = -std::numeric_limits<double>::infinity();
    double min_vertical = std::numeric_limits<double>::infinity();
    double max_vertical = -std::numeric_limits<double>::infinity();
    double min_distance = std::numeric_limits<double>::infinity();
    double max_distance = 0.0;
    double min_rotation = std::numeric_limits<double>::infinity();
    double max_rotation = 0.0;
    for (const auto& sample : samples) {
        const auto& t = sample.left_board_tvec;
        const double horizontal = std::atan2(t[0], t[2]);
        const double vertical = std::atan2(t[1], t[2]);
        const double distance = cv::norm(t);
        const double rotation = cv::norm(sample.left_board_rvec);
        min_horizontal = std::min(min_horizontal, horizontal);
        max_horizontal = std::max(max_horizontal, horizontal);
        min_vertical = std::min(min_vertical, vertical);
        max_vertical = std::max(max_vertical, vertical);
        min_distance = std::min(min_distance, distance);
        max_distance = std::max(max_distance, distance);
        min_rotation = std::min(min_rotation, rotation);
        max_rotation = std::max(max_rotation, rotation);
    }
    constexpr double ten_degrees = 10.0 * 3.14159265358979323846 / 180.0;
    constexpr double twenty_degrees = 20.0 * 3.14159265358979323846 / 180.0;
    const double position = 0.5 * (clamp01((max_horizontal - min_horizontal) / ten_degrees) +
                                   clamp01((max_vertical - min_vertical) / ten_degrees));
    const double mean_distance = 0.5 * (min_distance + max_distance);
    const double distance = mean_distance > 0.0
                                ? clamp01((max_distance - min_distance) / (0.25 * mean_distance))
                                : 0.0;
    return {position, distance,
            clamp01((max_rotation - min_rotation) / twenty_degrees)};
}

void drawObservation(cv::Mat& image,
                     const std::unordered_map<int, std::array<cv::Point2f, 4>>& markers,
                     const std::set<int>& common_ids,
                     const double scale) {
    for (const auto& [id, corners] : markers) {
        std::vector<cv::Point> polygon;
        polygon.reserve(4);
        for (const auto& corner : corners) {
            polygon.emplace_back(cvRound(corner.x * scale), cvRound(corner.y * scale));
        }
        const cv::Scalar color = common_ids.count(id) ? cv::Scalar(0, 220, 0)
                                                       : cv::Scalar(0, 210, 255);
        cv::polylines(image, polygon, true, color, 2, cv::LINE_AA);
        for (std::size_t corner = 0; corner < polygon.size(); ++corner) {
            cv::circle(image, polygon[corner], 4, color, cv::FILLED, cv::LINE_AA);
        }
        cv::putText(image, "ID " + std::to_string(id), polygon.front() + cv::Point(5, -7),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
    }
}

}  // namespace

StereoArucoCalibratorNode::StereoArucoCalibratorNode(const rclcpp::NodeOptions& options)
    : Node("stereo_aruco_calibrator", options),
      last_left_processed_(0, 0, RCL_ROS_TIME),
      last_right_processed_(0, 0, RCL_ROS_TIME) {
    const auto marker_ids_parameter =
        declare_parameter<std::vector<std::int64_t>>("board.marker_ids", {1, 2, 3, 4});
    if (marker_ids_parameter.size() != 4) {
        throw std::invalid_argument("board.marker_ids must contain exactly four IDs");
    }
    std::array<int, 4> marker_ids{};
    std::transform(marker_ids_parameter.begin(), marker_ids_parameter.end(), marker_ids.begin(),
                   [](const std::int64_t value) { return static_cast<int>(value); });
    board_ = std::make_unique<BoardModel>(
        declare_parameter<double>("board.marker_length_m", 0.20),
        declare_parameter<double>("board.center_spacing_x_m", 1.10),
        declare_parameter<double>("board.center_spacing_y_m", 0.70), marker_ids);

    dictionary_ = cv::aruco::getPredefinedDictionary(
        declare_parameter<int>("board.dictionary_id", 8));
    detector_parameters_ = cv::aruco::DetectorParameters::create();
    detector_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    detector_parameters_->cornerRefinementWinSize = 5;
    detector_parameters_->cornerRefinementMaxIterations = 30;
    detector_parameters_->cornerRefinementMinAccuracy = 0.01;

    minimum_common_markers_ = declare_parameter<int>("sampling.minimum_common_markers", 3);
    minimum_samples_ = declare_parameter<int>("sampling.minimum_samples", 10);
    maximum_samples_ = declare_parameter<int>("sampling.maximum_samples", 40);
    max_pair_delta_s_ = declare_parameter<double>("sampling.max_pair_delta_s", 0.005);
    maximum_reprojection_error_px_ =
        declare_parameter<double>("sampling.maximum_reprojection_error_px", 1.0);
    stability_minimum_duration_s_ =
        declare_parameter<double>("sampling.stability.minimum_duration_s", 0.4);
    stability_minimum_pairs_ = static_cast<std::size_t>(
        declare_parameter<int>("sampling.stability.minimum_pairs", 6));
    stability_maximum_corner_motion_px_ =
        declare_parameter<double>("sampling.stability.maximum_corner_motion_px", 1.0);
    stability_rearm_corner_motion_px_ =
        declare_parameter<double>("sampling.stability.rearm_corner_motion_px", 8.0);
    const double processing_rate_hz =
        declare_parameter<double>("sampling.processing_rate_hz", 15.0);
    queue_size_ = static_cast<std::size_t>(declare_parameter<int>("sampling.queue_size", 5));
    capturing_ = declare_parameter<bool>("sampling.auto_start", true);
    sampling_state_ = capturing_ ? "WAITING_FOR_MARKERS" : "PREVIEW_ONLY";

    preview_enabled_ = declare_parameter<bool>("display.preview_enabled", true);
    const double preview_rate_hz = declare_parameter<double>("display.preview_rate_hz", 5.0);
    preview_scale_ = declare_parameter<double>("display.preview_scale", 0.5);
    jpeg_quality_ = declare_parameter<int>("display.jpeg_quality", 80);
    const double status_rate_hz = declare_parameter<double>("display.status_rate_hz", 5.0);
    output_path_ = declare_parameter<std::string>("output_path", "/tmp/stereo_extrinsics.yaml");

    if (minimum_common_markers_ < 1 || minimum_common_markers_ > 4 || minimum_samples_ < 2 ||
        maximum_samples_ < minimum_samples_ || !(max_pair_delta_s_ >= 0.0) ||
        !(maximum_reprojection_error_px_ > 0.0) || !(processing_rate_hz > 0.0) ||
        !(stability_minimum_duration_s_ >= 0.0) || stability_minimum_pairs_ == 0 ||
        !(stability_maximum_corner_motion_px_ >= 0.0) ||
        !(stability_rearm_corner_motion_px_ > stability_maximum_corner_motion_px_) ||
        queue_size_ == 0 || !(preview_rate_hz > 0.0) || !(preview_scale_ > 0.0) ||
        preview_scale_ > 1.0 || jpeg_quality_ < 1 || jpeg_quality_ > 100 ||
        !(status_rate_hz > 0.0)) {
        throw std::invalid_argument("Invalid calibration or display parameters");
    }
    minimum_process_interval_s_ = 1.0 / processing_rate_hz;
    preview_interval_s_ = 1.0 / preview_rate_hz;
    sample_manager_ = std::make_unique<SampleManager>(SampleSelectionConfig{
        static_cast<std::size_t>(minimum_common_markers_ * 4),
        static_cast<std::size_t>(maximum_samples_),
        declare_parameter<double>("sampling.minimum_pose_rotation_deg", 5.0),
        declare_parameter<double>("sampling.minimum_pose_translation_m", 0.08)});
    stability_gate_ = std::make_unique<StabilityGate>(StabilityConfig{
        stability_minimum_duration_s_, stability_minimum_pairs_,
        stability_maximum_corner_motion_px_, stability_rearm_corner_motion_px_});

    const auto left_image_topic =
        declare_parameter<std::string>("left.image_topic", "/left_camera/image_raw");
    const auto right_image_topic =
        declare_parameter<std::string>("right.image_topic", "/right_camera/image_raw");
    const auto left_info_topic =
        declare_parameter<std::string>("left.camera_info_topic", "/left_camera/camera_info");
    const auto right_info_topic =
        declare_parameter<std::string>("right.camera_info_topic", "/right_camera/camera_info");

    preview_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "preview/compressed", rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("status", 10);
    status_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / status_rate_hz), [this] { publishStatus(); });
    preview_timer_ = create_wall_timer(
        std::chrono::duration<double>(preview_interval_s_), [this] { publishLatestPreview(); });

    left_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        left_info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
            cameraInfoCallback(std::move(message), true);
        });
    right_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        right_info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
            cameraInfoCallback(std::move(message), false);
        });
    left_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        left_image_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
            imageCallback(std::move(message), true);
        });
    right_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        right_image_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
            imageCallback(std::move(message), false);
        });

    const auto simple_service = [this](const std::string& name, const bool capture,
                                       const bool clear) {
        return create_service<std_srvs::srv::Trigger>(
            name, [this, capture, clear](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (clear) {
                    sample_manager_->clear();
                    last_result_.reset();
                }
                left_queue_.clear();
                right_queue_.clear();
                stability_gate_->reset();
                stability_progress_ = 0.0;
                stability_pair_count_ = 0;
                stability_duration_ms_ = 0.0;
                sampling_state_ = capture ? "WAITING_FOR_MARKERS" : "PREVIEW_ONLY";
                capturing_ = capture;
                last_decision_ = clear ? "samples_cleared" :
                                 (capture ? "capture_started" : "capture_stopped");
                response->success = true;
                response->message = last_decision_;
            });
    };
    start_service_ = simple_service("start", true, false);
    stop_service_ = simple_service("stop", false, false);
    reset_service_ = simple_service("reset", false, true);

    solve_service_ = create_service<std_srvs::srv::Trigger>(
        "solve", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            std::vector<StereoSample> samples;
            CameraIntrinsics left;
            CameraIntrinsics right;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                samples = sample_manager_->samples();
                left = left_intrinsics_;
                right = right_intrinsics_;
                if (samples.size() < static_cast<std::size_t>(minimum_samples_)) {
                    response->success = false;
                    response->message = "not enough samples: " + std::to_string(samples.size()) +
                                        "/" + std::to_string(minimum_samples_);
                    last_decision_ = "not_enough_samples";
                    return;
                }
                solving_ = true;
                capturing_ = false;
                left_queue_.clear();
                right_queue_.clear();
                stability_gate_->reset();
                sampling_state_ = "PREVIEW_ONLY";
                last_decision_ = "solving";
            }
            StereoCalibrationResult result;
            std::string error;
            const bool success = StereoSolver::solve(samples, left, right, result, error);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                solving_ = false;
                if (success) {
                    last_result_ = result;
                    last_decision_ = "solved";
                } else {
                    last_decision_ = "solve_failed";
                }
            }
            response->success = success;
            response->message = success
                ? "solved: rms=" + number(result.stereo_rms_px) +
                      " px, baseline=" + number(result.baseline_m) + " m"
                : error;
            RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        });

    save_service_ = create_service<std_srvs::srv::Trigger>(
        "save", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            std::optional<StereoCalibrationResult> result;
            CameraIntrinsics left;
            CameraIntrinsics right;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                result = last_result_;
                left = left_intrinsics_;
                right = right_intrinsics_;
            }
            if (!result) {
                response->success = false;
                response->message = "no solved calibration is available";
                return;
            }
            std::string error;
            response->success = ResultWriter::write(output_path_, left, right, *result, error);
            response->message = response->success ? "saved to " + output_path_ : error;
            std::lock_guard<std::mutex> lock(mutex_);
            last_decision_ = response->success ? "saved" : "save_failed";
        });

    RCLCPP_INFO(get_logger(),
                "Stereo ArUco calibrator ready; capture=%s, preview=%s, output='%s'",
                capturing_ ? "on" : "off", preview_enabled_ ? "on" : "off",
                output_path_.c_str());
}

void StereoArucoCalibratorNode::setDecision(const std::string& decision) {
    last_decision_ = decision;
}

void StereoArucoCalibratorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message, const bool is_left) {
    if (message->distortion_model != "plumb_bob" || message->width == 0 || message->height == 0 ||
        message->header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Ignoring invalid CameraInfo on %s side", is_left ? "left" : "right");
        return;
    }
    auto intrinsics = intrinsicsFrom(*message);
    if (!intrinsics.valid()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Ignoring uncalibrated CameraInfo on %s side", is_left ? "left" : "right");
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto& destination = is_left ? left_intrinsics_ : right_intrinsics_;
    if (destination.valid() &&
        (destination.image_size != intrinsics.image_size ||
         cv::norm(destination.camera_matrix - intrinsics.camera_matrix) > 1e-9 ||
         destination.frame_id != intrinsics.frame_id)) {
        sample_manager_->clear();
        last_result_.reset();
        setDecision("camera_info_changed_samples_cleared");
        RCLCPP_WARN(get_logger(), "CameraInfo changed; collected samples were cleared");
    }
    destination = std::move(intrinsics);
}

void StereoArucoCalibratorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& message, const bool is_left) {
    const rclcpp::Time stamp(message->header.stamp);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& last_stamp = is_left ? last_left_processed_ : last_right_processed_;
        if (last_stamp.nanoseconds() != 0 && stamp > last_stamp &&
            (stamp - last_stamp).seconds() < minimum_process_interval_s_) {
            return;
        }
        last_stamp = stamp;
    }
    auto observation = detect(message);
    if (observation) {
        queueObservation(std::move(*observation), is_left);
    }
}

std::optional<StereoArucoCalibratorNode::Observation> StereoArucoCalibratorNode::detect(
    const sensor_msgs::msg::Image::ConstSharedPtr& message) {
    try {
        const auto image = cv_bridge::toCvShare(message, "bgr8");
        cv::Mat gray;
        cv::cvtColor(image->image, gray, cv::COLOR_BGR2GRAY);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<std::vector<cv::Point2f>> rejected;
        cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_parameters_, rejected);
        Observation observation{rclcpp::Time(message->header.stamp), message->header.frame_id,
                                cv::Size(static_cast<int>(message->width),
                                         static_cast<int>(message->height)), {}, {}};
        for (std::size_t index = 0; index < ids.size(); ++index) {
            if (board_->cornersForMarker(ids[index]) && corners[index].size() == 4) {
                observation.markers.emplace(
                    ids[index], std::array<cv::Point2f, 4>{{corners[index][0], corners[index][1],
                                                            corners[index][2], corners[index][3]}});
            }
        }
        if (preview_enabled_ && preview_publisher_->get_subscription_count() > 0) {
            cv::resize(image->image, observation.preview, cv::Size(), preview_scale_, preview_scale_,
                       cv::INTER_AREA);
        }
        return observation;
    } catch (const std::exception& exception) {
        std::lock_guard<std::mutex> lock(mutex_);
        setDecision("image_conversion_or_detection_failed");
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "ArUco detection failed: %s", exception.what());
        return std::nullopt;
    }
}

void StereoArucoCalibratorNode::queueObservation(Observation observation, const bool is_left) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& latest = is_left ? latest_left_observation_ : latest_right_observation_;
    latest = observation;
    if (!capturing_) {
        return;
    }
    const auto& intrinsics = is_left ? left_intrinsics_ : right_intrinsics_;
    if (!intrinsics.valid()) {
        setDecision(is_left ? "left_camera_info_missing" : "right_camera_info_missing");
        sampling_state_ = "WAITING_FOR_CAMERA_INFO";
        return;
    }
    if (intrinsics.image_size != observation.image_size ||
        intrinsics.frame_id != observation.frame_id) {
        setDecision(is_left ? "left_image_camera_info_mismatch"
                            : "right_image_camera_info_mismatch");
        sampling_state_ = "WAITING_FOR_CAMERA_INFO";
        return;
    }
    auto& queue = is_left ? left_queue_ : right_queue_;
    queue.push_back(std::move(observation));
    while (queue.size() > queue_size_) {
        queue.pop_front();
    }
    matchObservations();
}

void StereoArucoCalibratorNode::matchObservations() {
    while (!left_queue_.empty() && !right_queue_.empty()) {
        std::size_t best_left = 0;
        std::size_t best_right = 0;
        double best_delta = std::numeric_limits<double>::infinity();
        for (std::size_t left_index = 0; left_index < left_queue_.size(); ++left_index) {
            for (std::size_t right_index = 0; right_index < right_queue_.size(); ++right_index) {
                const double delta = std::abs((left_queue_[left_index].stamp -
                                               right_queue_[right_index].stamp).seconds());
                if (delta < best_delta) {
                    best_delta = delta;
                    best_left = left_index;
                    best_right = right_index;
                }
            }
        }
        last_pair_delta_ms_ = best_delta * 1000.0;
        if (best_delta <= max_pair_delta_s_) {
            const auto left = std::move(left_queue_[best_left]);
            const auto right = std::move(right_queue_[best_right]);
            left_queue_.erase(left_queue_.begin(), left_queue_.begin() + best_left + 1);
            right_queue_.erase(right_queue_.begin(), right_queue_.begin() + best_right + 1);
            processPair(left, right);
            continue;
        }
        sampling_state_ = "WAITING_FOR_SYNC";
        stability_progress_ = 0.0;
        stability_gate_->reset();
        setDecision("pair_delta_too_large");
        if (left_queue_.front().stamp < right_queue_.front().stamp) {
            left_queue_.pop_front();
        } else {
            right_queue_.pop_front();
        }
    }
}

void StereoArucoCalibratorNode::processPair(const Observation& left,
                                             const Observation& right) {
    std::vector<int> common_ids;
    for (const auto& item : left.markers) {
        if (right.markers.count(item.first) != 0U) {
            common_ids.push_back(item.first);
        }
    }
    std::sort(common_ids.begin(), common_ids.end());
    last_left_marker_count_ = left.markers.size();
    last_right_marker_count_ = right.markers.size();
    last_common_marker_count_ = common_ids.size();
    last_pair_delta_ms_ = std::abs((left.stamp - right.stamp).seconds()) * 1000.0;
    if (common_ids.size() < static_cast<std::size_t>(minimum_common_markers_)) {
        sampling_state_ = "WAITING_FOR_MARKERS";
        stability_progress_ = 0.0;
        stability_gate_->reset();
        setDecision(common_ids.empty() ? "no_common_markers" : "insufficient_common_markers");
        return;
    }

    StereoSample sample;
    sample.stamp_delta_s = last_pair_delta_ms_ / 1000.0;
    for (const int id : common_ids) {
        const auto object_corners = board_->cornersForMarker(id);
        const auto& left_corners = left.markers.at(id);
        const auto& right_corners = right.markers.at(id);
        for (std::size_t corner = 0; corner < 4; ++corner) {
            sample.object_points.push_back((*object_corners)[corner]);
            sample.left_points.push_back(left_corners[corner]);
            sample.right_points.push_back(right_corners[corner]);
        }
    }

    const double pair_stamp_s = 0.5 * (left.stamp.seconds() + right.stamp.seconds());
    const auto stability = stability_gate_->update(
        pair_stamp_s, common_ids, sample.left_points, sample.right_points);
    stability_progress_ = stability.progress;
    stability_pair_count_ = stability.pair_count;
    stability_duration_ms_ = stability.duration_s * 1000.0;
    last_left_corner_motion_px_ = stability.left_motion_px;
    last_right_corner_motion_px_ = stability.right_motion_px;
    if (stability.waiting_for_motion) {
        sampling_state_ = "WAITING_FOR_MOTION";
        setDecision("waiting_for_motion");
        return;
    }
    if (!stability.ready) {
        sampling_state_ = "STABILIZING";
        setDecision("stabilizing");
        return;
    }
    sampling_state_ = "STABLE";
    const auto consume_stable_pose = [this, &common_ids, &sample] {
        stability_gate_->consume(common_ids, sample.left_points, sample.right_points);
        sampling_state_ = "WAITING_FOR_MOTION";
        stability_progress_ = 0.0;
    };

    cv::Vec3d left_rvec;
    cv::Vec3d left_tvec;
    cv::Vec3d right_rvec;
    cv::Vec3d right_tvec;
    try {
        const bool left_ok = cv::solvePnP(sample.object_points, sample.left_points,
                                          left_intrinsics_.camera_matrix, left_intrinsics_.distortion,
                                          left_rvec, left_tvec, false, cv::SOLVEPNP_ITERATIVE);
        const bool right_ok = cv::solvePnP(sample.object_points, sample.right_points,
                                           right_intrinsics_.camera_matrix, right_intrinsics_.distortion,
                                           right_rvec, right_tvec, false, cv::SOLVEPNP_ITERATIVE);
        if (!left_ok || !right_ok || left_tvec[2] <= 0.0 || right_tvec[2] <= 0.0) {
            consume_stable_pose();
            setDecision("pnp_failed_or_board_behind_camera");
            return;
        }
        last_left_reprojection_px_ = meanReprojectionError(
            sample.object_points, sample.left_points, left_rvec, left_tvec, left_intrinsics_);
        last_right_reprojection_px_ = meanReprojectionError(
            sample.object_points, sample.right_points, right_rvec, right_tvec, right_intrinsics_);
        if (last_left_reprojection_px_ > maximum_reprojection_error_px_ ||
            last_right_reprojection_px_ > maximum_reprojection_error_px_) {
            consume_stable_pose();
            setDecision("reprojection_error_too_large");
            return;
        }
    } catch (const cv::Exception&) {
        consume_stable_pose();
        setDecision("pnp_exception");
        return;
    }

    sample.left_board_rvec = left_rvec;
    sample.left_board_tvec = left_tvec;
    const double stamp_delta_ms = sample.stamp_delta_s * 1000.0;
    std::string reason;
    consume_stable_pose();
    if (!sample_manager_->addIfDiverse(std::move(sample), reason)) {
        setDecision(reason == "pose is too similar to an existing sample"
                        ? "pose_too_similar"
                        : "sample_rejected");
        return;
    }

    last_result_.reset();
    setDecision("accepted");
    RCLCPP_INFO(get_logger(), "Accepted stereo sample %zu/%d (%zu common markers, dt=%.3f ms)",
                sample_manager_->size(), maximum_samples_, common_ids.size(), stamp_delta_ms);
    if (sample_manager_->full()) {
        capturing_ = false;
        sampling_state_ = "PREVIEW_ONLY";
        setDecision("sample_limit_reached");
        RCLCPP_INFO(get_logger(), "Sample limit reached; capture stopped. Call the solve service.");
    }
}

void StereoArucoCalibratorNode::publishLatestPreview() {
    if (!preview_enabled_ || preview_publisher_->get_subscription_count() == 0) {
        return;
    }
    std::optional<Observation> left;
    std::optional<Observation> right;
    bool capturing = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        left = latest_left_observation_;
        right = latest_right_observation_;
        capturing = capturing_;
    }
    if (!left || !right) {
        return;
    }
    std::vector<int> common_ids;
    for (const auto& item : left->markers) {
        if (right->markers.count(item.first) != 0U) {
            common_ids.push_back(item.first);
        }
    }
    publishPreview(*left, *right, common_ids, capturing);
}

void StereoArucoCalibratorNode::publishPreview(const Observation& left,
                                                const Observation& right,
                                                const std::vector<int>& common_ids,
                                                const bool capturing) {
    if (!preview_enabled_ || preview_publisher_->get_subscription_count() == 0 ||
        left.preview.empty() || right.preview.empty()) {
        return;
    }
    cv::Mat left_image = left.preview.clone();
    cv::Mat right_image = right.preview.clone();
    const std::set<int> common(common_ids.begin(), common_ids.end());
    drawObservation(left_image, left.markers, common, preview_scale_);
    drawObservation(right_image, right.markers, common, preview_scale_);
    cv::putText(left_image, "LEFT", {18, 34}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                {255, 255, 255}, 2, cv::LINE_AA);
    cv::putText(right_image, "RIGHT", {18, 34}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                {255, 255, 255}, 2, cv::LINE_AA);
    const double delta_ms = std::abs((left.stamp - right.stamp).seconds()) * 1000.0;
    const std::string mode = capturing ? "CAPTURE ON" : "PREVIEW ONLY";
    cv::putText(left_image, mode, {18, 68}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                capturing ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 190, 255), 2, cv::LINE_AA);
    if (delta_ms > max_pair_delta_s_ * 1000.0) {
        cv::putText(right_image, "UNSYNC " + number(delta_ms, 1) + " ms", {18, 68},
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 0, 255}, 2, cv::LINE_AA);
    }

    const int height = std::max(left_image.rows, right_image.rows);
    cv::Mat composite(height, left_image.cols + right_image.cols, CV_8UC3, cv::Scalar(20, 20, 20));
    left_image.copyTo(composite(cv::Rect(0, 0, left_image.cols, left_image.rows)));
    right_image.copyTo(composite(cv::Rect(left_image.cols, 0, right_image.cols, right_image.rows)));
    cv::line(composite, {left_image.cols, 0}, {left_image.cols, height}, {255, 255, 255}, 1);

    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = left.stamp;
    message.header.frame_id = "stereo_calibration_preview";
    message.format = "jpeg";
    cv::imencode(".jpg", composite, message.data,
                 {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    preview_publisher_->publish(std::move(message));
}

void StereoArucoCalibratorNode::publishStatus() {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "stereo_calibration/status";
    status.hardware_id = "stereo_aruco_board";

    std::lock_guard<std::mutex> lock(mutex_);
    const auto samples = sample_manager_->samples();
    const auto coverage = calculateCoverage(samples);
    const bool can_solve = samples.size() >= static_cast<std::size_t>(minimum_samples_) && !solving_;
    const bool can_save = last_result_.has_value() && !solving_;
    const std::string state = solving_ ? "SOLVING" :
                              capturing_ ? "CAPTURING" :
                              last_result_ ? "SOLVED" :
                              can_solve ? "READY" : "IDLE";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = state;
    status.values = {
        keyValue("state", state),
        keyValue("accepted_samples", std::to_string(samples.size())),
        keyValue("minimum_samples", std::to_string(minimum_samples_)),
        keyValue("maximum_samples", std::to_string(maximum_samples_)),
        keyValue("sampling_state", sampling_state_),
        keyValue("stability_progress", number(stability_progress_)),
        keyValue("stability_pair_count", std::to_string(stability_pair_count_)),
        keyValue("stability_duration_ms", number(stability_duration_ms_)),
        keyValue("left_corner_motion_px", number(last_left_corner_motion_px_)),
        keyValue("right_corner_motion_px", number(last_right_corner_motion_px_)),
        keyValue("left_marker_count", std::to_string(last_left_marker_count_)),
        keyValue("right_marker_count", std::to_string(last_right_marker_count_)),
        keyValue("common_marker_count", std::to_string(last_common_marker_count_)),
        keyValue("pair_delta_ms", number(last_pair_delta_ms_)),
        keyValue("left_reprojection_error_px", number(last_left_reprojection_px_)),
        keyValue("right_reprojection_error_px", number(last_right_reprojection_px_)),
        keyValue("sample_progress", number(static_cast<double>(samples.size()) /
                                            static_cast<double>(maximum_samples_))),
        keyValue("position_coverage", number(coverage.position)),
        keyValue("distance_coverage", number(coverage.distance)),
        keyValue("rotation_coverage", number(coverage.rotation)),
        keyValue("can_solve", can_solve ? "true" : "false"),
        keyValue("can_save", can_save ? "true" : "false"),
        keyValue("last_decision", last_decision_),
    };
    if (last_result_) {
        status.values.push_back(keyValue("stereo_rms_px", number(last_result_->stereo_rms_px)));
        status.values.push_back(keyValue("mean_epipolar_error_px",
                                         number(last_result_->mean_epipolar_error_px)));
        status.values.push_back(keyValue("max_epipolar_error_px",
                                         number(last_result_->max_epipolar_error_px)));
        status.values.push_back(keyValue("baseline_m", number(last_result_->baseline_m, 6)));
    }
    message.status.push_back(std::move(status));
    status_publisher_->publish(std::move(message));
}

}  // namespace stereo_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_calibration::StereoArucoCalibratorNode)
