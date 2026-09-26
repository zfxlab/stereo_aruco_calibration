#include "stereo_aruco_calibration/stability_gate.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace stereo_calibration {
namespace {

std::vector<cv::Point2f> corners(float offset_x = 0.0F) {
    return {{offset_x + 0.0F, 0.0F}, {offset_x + 1.0F, 0.0F},
            {offset_x + 1.0F, 1.0F}, {offset_x + 0.0F, 1.0F}};
}

TEST(StabilityGate, RequiresBothDurationAndPairCount) {
    StabilityGate gate({0.4, 3, 1.0, 8.0});
    const std::vector<int> ids{1};
    EXPECT_FALSE(gate.update(0.0, ids, corners(), corners()).ready);
    EXPECT_FALSE(gate.update(0.2, ids, corners(0.2F), corners(0.2F)).ready);
    const auto status = gate.update(0.4, ids, corners(0.1F), corners(0.1F));
    EXPECT_TRUE(status.ready);
    EXPECT_DOUBLE_EQ(status.progress, 1.0);
}

TEST(StabilityGate, ContinuousDriftResetsTheWholeWindow) {
    StabilityGate gate({0.4, 3, 1.0, 8.0});
    const std::vector<int> ids{1};
    EXPECT_FALSE(gate.update(0.0, ids, corners(), corners()).ready);
    EXPECT_FALSE(gate.update(0.2, ids, corners(0.6F), corners(0.6F)).ready);
    const auto reset = gate.update(0.4, ids, corners(1.2F), corners(1.2F));
    EXPECT_FALSE(reset.ready);
    EXPECT_EQ(reset.pair_count, 1U);
    EXPECT_DOUBLE_EQ(reset.duration_s, 0.0);
}

TEST(StabilityGate, ConsumedPoseMustMoveBeforeRearming) {
    StabilityGate gate({0.2, 2, 1.0, 8.0});
    const std::vector<int> ids{1};
    gate.update(0.0, ids, corners(), corners());
    ASSERT_TRUE(gate.update(0.2, ids, corners(), corners()).ready);
    gate.consume(ids, corners(), corners());

    const auto waiting = gate.update(0.3, ids, corners(2.0F), corners(2.0F));
    EXPECT_TRUE(waiting.waiting_for_motion);
    const auto rearmed = gate.update(0.4, ids, corners(8.0F), corners(8.0F));
    EXPECT_FALSE(rearmed.waiting_for_motion);
    EXPECT_FALSE(rearmed.ready);
    EXPECT_EQ(rearmed.pair_count, 1U);
}

TEST(StabilityGate, MarkerSetChangeStartsANewWindow) {
    StabilityGate gate({0.2, 2, 1.0, 8.0});
    gate.update(0.0, {1}, corners(), corners());
    const auto status = gate.update(0.2, {2}, corners(), corners());
    EXPECT_FALSE(status.ready);
    EXPECT_EQ(status.pair_count, 1U);
}

}  // namespace
}  // namespace stereo_calibration
