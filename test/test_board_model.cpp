#include "stereo_aruco_calibration/board_model.hpp"
#include <gtest/gtest.h>

TEST(BoardModel, UsesCenterSpacingAndOpenCvCornerOrder) {
    const stereo_calibration::BoardModel board(0.20, 1.10, 0.70, {1, 2, 3, 4});
    const auto corners = board.cornersForMarker(1);
    ASSERT_TRUE(corners.has_value());
    EXPECT_NEAR((*corners)[0].x, -0.65, 1e-6);
    EXPECT_NEAR((*corners)[0].y, -0.45, 1e-6);
    EXPECT_NEAR((*corners)[1].x, -0.45, 1e-6);
    EXPECT_NEAR((*corners)[2].y, -0.25, 1e-6);
    EXPECT_FALSE(board.cornersForMarker(99).has_value());
}

TEST(BoardModel, RejectsDuplicateIds) {
    EXPECT_THROW((stereo_calibration::BoardModel(0.20, 1.10, 0.70, {1, 1, 3, 4})),
                 std::invalid_argument);
}
