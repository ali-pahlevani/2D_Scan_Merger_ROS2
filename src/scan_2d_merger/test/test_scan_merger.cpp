/**
 * Copyright 2026 Ali Pahlevani
 *
 * Integration tests for LaserScanMerger. Each test spins the real node with
 * synthetic scans and a static TF, then checks the merged output numerically.
 */
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "scan_2d_merger/scan_2d_merger.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// Incremented per test to generate unique node and topic names.
static std::atomic<int> g_tc{0};

// Returns a LaserScan filled with +inf; caller sets individual ranges as needed.
static sensor_msgs::msg::LaserScan makeScan(
  const std::string& frame_id,
  rclcpp::Time       stamp,
  float ang_min   = static_cast<float>(-M_PI),
  float ang_max   = static_cast<float>(M_PI),
  float ang_inc   = static_cast<float>(M_PI / 180.0),
  float range_min = 0.1f,
  float range_max = 100.0f)
{
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id = frame_id;
  s.header.stamp    = stamp;
  s.angle_min       = ang_min;
  s.angle_max       = ang_max;
  s.angle_increment = ang_inc;
  s.time_increment  = 0.0f;
  s.scan_time       = 1.0f / 30.0f;
  s.range_min       = range_min;
  s.range_max       = range_max;

  const auto n = static_cast<uint32_t>(std::ceil((ang_max - ang_min) / ang_inc));
  s.ranges.assign(n, std::numeric_limits<float>::infinity());
  return s;
}

// Returns a TransformStamped from parent to child with the given translation and quaternion.
static geometry_msgs::msg::TransformStamped makeTransform(
  const std::string& parent, const std::string& child,
  double tx, double ty, double tz,
  double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id  = parent;
  ts.child_frame_id   = child;
  ts.transform.translation.x = tx;
  ts.transform.translation.y = ty;
  ts.transform.translation.z = tz;
  ts.transform.rotation.x = qx;
  ts.transform.rotation.y = qy;
  ts.transform.rotation.z = qz;
  ts.transform.rotation.w = qw;
  return ts;
}

class ScanMergerTest : public ::testing::Test
{
protected:
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Node::SharedPtr                  helper_;
  std::string                              merged_topic_;

  void SetUp() override
  {
    int id = g_tc.fetch_add(1);
    merged_topic_ = "/merged_test_" + std::to_string(id);
    helper_ = rclcpp::Node::make_shared("helper_" + std::to_string(id));
    exec_.add_node(helper_);
  }

  void TearDown() override { exec_.remove_node(helper_); }

  // Spin until pred returns true or the timeout elapses; returns whether pred was met.
  bool spinUntil(std::function<bool()> pred, std::chrono::milliseconds timeout = 4000ms)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_some(10ms);
      if (pred()) return true;
    }
    return false;
  }

  // Creates a LaserScanMerger with standard test parameters; adds it to the executor.
  std::shared_ptr<util::LaserScanMerger> makeMerger(
    const std::vector<std::string>& topics,
    const std::string& merged_frame = "base_link",
    double sync_slop  = 0.5,
    double range_min  = 0.1,
    double range_max  = 100.0,
    double min_height = -100.0,
    double max_height =  100.0)
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("scan_topics",     topics);
    opts.append_parameter_override("merged_frame_id", merged_frame);
    opts.append_parameter_override("output_topic",    merged_topic_);
    opts.append_parameter_override("sync_slop",       sync_slop);
    opts.append_parameter_override("range_min",       range_min);
    opts.append_parameter_override("range_max",       range_max);
    opts.append_parameter_override("min_height",      min_height);
    opts.append_parameter_override("max_height",      max_height);
    opts.append_parameter_override("angle_min",       -M_PI);
    opts.append_parameter_override("angle_max",        M_PI);
    opts.append_parameter_override("angle_increment",  M_PI / 180.0);
    opts.append_parameter_override("tolerance",        5.0);
    opts.append_parameter_override("use_inf",          true);
    opts.append_parameter_override("queue_size",       10);
    opts.append_parameter_override("moving_frames",    false);
    opts.append_parameter_override("debug",            false);
    opts.append_parameter_override("scan_time",        1.0 / 30.0);
    opts.append_parameter_override("inf_epsilon",      1.0);

    auto node = std::make_shared<util::LaserScanMerger>(opts);
    exec_.add_node(node);
    return node;
  }

  void removeMerger(std::shared_ptr<util::LaserScanMerger> node)
  {
    exec_.remove_node(node);
  }

  // Broadcasts a static TF and spins ~300 ms to let the merger's buffer pick it up.
  void broadcastTF(const geometry_msgs::msg::TransformStamped& ts)
  {
    auto bc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    bc->sendTransform(ts);
    auto deadline = std::chrono::steady_clock::now() + 300ms;
    while (std::chrono::steady_clock::now() < deadline)
      exec_.spin_some(10ms);
  }
};

// Maps an angle in the merged frame to the expected output bin index,
// mirroring the formula in projectScan so test expectations stay consistent.
static int angleToBin(double angle_rad,
                      double ang_min = -M_PI,
                      double ang_inc = M_PI / 180.0)
{
  return static_cast<int>((angle_rad - ang_min) / ang_inc);
}

// Single LiDAR, identity transform: a ray at index 180 (angle 0°) must land in bin 180.
TEST_F(ScanMergerTest, SingleLidar_IdentityTransform_CorrectBin)
{
  auto merger = makeMerger({"/t1/scan"});
  broadcastTF(makeTransform("base_link", "lidar_link", 0, 0, 0, 0, 0, 0, 1));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t1/scan", 10);
  spinUntil([&]{ return pub->get_subscription_count() > 0; });

  auto scan = makeScan("lidar_link", helper_->now());
  scan.ranges[180] = 2.5f;  // index 180 = 0 deg
  pub->publish(scan);

  ASSERT_TRUE(spinUntil([&]{ return !received.empty(); })) << "No merged scan received";

  const auto& out = received.front();
  ASSERT_EQ(out.ranges.size(), 360u);

  const int expected_bin = angleToBin(0.0);
  EXPECT_NEAR(out.ranges[expected_bin], 2.5f, 0.02f) << "Range at 0-deg bin should be 2.5 m";

  for (int b = 0; b < 360; ++b) {
    if (b != expected_bin) {
      EXPECT_TRUE(std::isinf(out.ranges[b])) << "Bin " << b << " should be inf";
    }
  }

  removeMerger(merger);
}

// Single LiDAR, 90 deg CCW rotation: point (3,0,0) -> (0,3,0), merged angle = pi/2 -> bin 270.
TEST_F(ScanMergerTest, SingleLidar_90DegRotation_CorrectBin)
{
  auto merger = makeMerger({"/t2/scan"});

  // 90 deg CCW around Z: qz = sin(45 deg), qw = cos(45 deg)
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  broadcastTF(makeTransform("base_link", "lidar_link", 0, 0, 0, 0, 0, s, c));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t2/scan", 10);
  spinUntil([&]{ return pub->get_subscription_count() > 0; });

  auto scan = makeScan("lidar_link", helper_->now());
  scan.ranges[180] = 3.0f;  // 0 deg in lidar frame; after rotation lands at 90 deg (bin 270)
  pub->publish(scan);

  ASSERT_TRUE(spinUntil([&]{ return !received.empty(); })) << "No merged scan received";

  const int expected_bin = angleToBin(M_PI / 2.0);
  EXPECT_NEAR(received.front().ranges[expected_bin], 3.0f, 0.02f)
    << "Range at 90-deg bin should be 3.0 m";
  EXPECT_TRUE(std::isinf(received.front().ranges[180]))
    << "Original 0-deg bin should be empty after rotation";

  removeMerger(merger);
}

// Two LiDARs facing the same direction: the closer reading must win at each bin.
TEST_F(ScanMergerTest, TwoLidars_SameDirection_MinRangeWins)
{
  auto merger = makeMerger({"/t3/scan0", "/t3/scan1"});
  broadcastTF(makeTransform("base_link", "lidar0_link", 0, 0, 0, 0, 0, 0, 1));
  broadcastTF(makeTransform("base_link", "lidar1_link", 0, 0, 0, 0, 0, 0, 1));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub0 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t3/scan0", 10);
  auto pub1 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t3/scan1", 10);
  spinUntil([&]{
    return pub0->get_subscription_count() > 0 && pub1->get_subscription_count() > 0;
  });

  auto now = helper_->now();
  auto s0 = makeScan("lidar0_link", now);  s0.ranges[180] = 5.0f;
  auto s1 = makeScan("lidar1_link", now);  s1.ranges[180] = 2.0f;
  pub0->publish(s0);
  pub1->publish(s1);

  ASSERT_TRUE(spinUntil([&]{ return !received.empty(); })) << "No merged scan received";
  EXPECT_NEAR(received.front().ranges[angleToBin(0.0)], 2.0f, 0.02f)
    << "Closer reading (2.0 m) should win over farther (5.0 m)";

  removeMerger(merger);
}

// Two LiDARs with complementary FOV: each contributes its half to the merged scan.
TEST_F(ScanMergerTest, TwoLidars_ComplementaryFOV_BothHalvesCovered)
{
  auto merger = makeMerger({"/t4/scan0", "/t4/scan1"});

  broadcastTF(makeTransform("base_link", "lidar0_link", 0, 0, 0, 0, 0, 0, 1));  // faces forward
  broadcastTF(makeTransform("base_link", "lidar1_link", 0, 0, 0, 0, 0, 1, 0));  // 180 deg around Z

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub0 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t4/scan0", 10);
  auto pub1 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t4/scan1", 10);
  spinUntil([&]{
    return pub0->get_subscription_count() > 0 && pub1->get_subscription_count() > 0;
  });

  // scan0 index 180 = 0 deg -> bin 180 in world.
  // scan1 index 270 = 90 deg in its frame; after 180 deg rotation: (0,6,0) -> (0,-6,0) = -90 deg -> bin 90.
  // Index 270 is used instead of 180 to avoid the +-180 deg boundary edge case.
  auto now = helper_->now();
  auto s0 = makeScan("lidar0_link", now);  s0.ranges[180] = 4.0f;
  auto s1 = makeScan("lidar1_link", now);  s1.ranges[270] = 6.0f;
  pub0->publish(s0);
  pub1->publish(s1);

  ASSERT_TRUE(spinUntil([&]{ return !received.empty(); })) << "No merged scan received";

  const auto& out = received.front();
  EXPECT_NEAR(out.ranges[angleToBin(0.0)],       4.0f, 0.02f) << "scan0 at bin 180 (0 deg)";
  EXPECT_NEAR(out.ranges[angleToBin(-M_PI/2.0)], 6.0f, 0.02f) << "scan1 at bin 90 (-90 deg)";

  removeMerger(merger);
}

// Scans within sync_slop should be fused; those outside must not produce output.
TEST_F(ScanMergerTest, TwoLidars_SyncSlop_WithinSlop_Merged)
{
  auto merger = makeMerger({"/t5a/scan0", "/t5a/scan1"}, "base_link", 0.1);
  broadcastTF(makeTransform("base_link", "l0", 0, 0, 0, 0, 0, 0, 1));
  broadcastTF(makeTransform("base_link", "l1", 0, 0, 0, 0, 0, 0, 1));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub0 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t5a/scan0", 10);
  auto pub1 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t5a/scan1", 10);
  spinUntil([&]{
    return pub0->get_subscription_count() > 0 && pub1->get_subscription_count() > 0;
  });

  rclcpp::Time t0 = helper_->now();
  rclcpp::Time t1 = t0 + rclcpp::Duration::from_seconds(0.05);  // 0.05 s < slop 0.1 s

  auto s0 = makeScan("l0", t0);  s0.ranges[180] = 1.5f;
  auto s1 = makeScan("l1", t1);  s1.ranges[90]  = 2.5f;
  pub0->publish(s0);
  pub1->publish(s1);

  EXPECT_TRUE(spinUntil([&]{ return !received.empty(); }))
    << "Scans within sync_slop should produce merged output";

  removeMerger(merger);
}

TEST_F(ScanMergerTest, TwoLidars_SyncSlop_OutsideSlop_NoOutput)
{
  auto merger = makeMerger({"/t5b/scan0", "/t5b/scan1"}, "base_link", 0.1);
  broadcastTF(makeTransform("base_link", "l0", 0, 0, 0, 0, 0, 0, 1));
  broadcastTF(makeTransform("base_link", "l1", 0, 0, 0, 0, 0, 0, 1));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub0 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t5b/scan0", 10);
  auto pub1 = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t5b/scan1", 10);
  spinUntil([&]{
    return pub0->get_subscription_count() > 0 && pub1->get_subscription_count() > 0;
  });

  rclcpp::Time t0 = helper_->now();
  rclcpp::Time t1 = t0 + rclcpp::Duration::from_seconds(1.0);  // 1.0 s >> slop 0.1 s

  pub0->publish(makeScan("l0", t0));
  pub1->publish(makeScan("l1", t1));

  EXPECT_FALSE(spinUntil([&]{ return !received.empty(); }, 1000ms))
    << "Out-of-slop scans must not produce output";

  removeMerger(merger);
}

// Readings outside [range_min, range_max] must be dropped; valid readings must pass through.
TEST_F(ScanMergerTest, SingleLidar_RangeFilter_OutOfRangeDropped)
{
  auto merger = makeMerger({"/t6/scan"}, "base_link", 0.5, 1.0, 10.0);
  broadcastTF(makeTransform("base_link", "lidar_link", 0, 0, 0, 0, 0, 0, 1));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t6/scan", 10);
  spinUntil([&]{ return pub->get_subscription_count() > 0; });

  auto scan = makeScan("lidar_link", helper_->now(), -M_PIf, M_PIf, M_PIf / 180.0f, 1.0f, 10.0f);
  scan.ranges[60]  = 0.05f;  // below range_min -> filtered
  scan.ranges[120] = 5.0f;   // valid
  scan.ranges[240] = 50.0f;  // above range_max -> filtered
  pub->publish(scan);

  ASSERT_TRUE(spinUntil([&]{ return !received.empty(); })) << "No merged scan received";

  const auto& out = received.front();
  EXPECT_TRUE(std::isinf(out.ranges[60]))    << "Too-close ray should be filtered";
  EXPECT_NEAR(out.ranges[120], 5.0f, 0.02f)  << "Valid ray should pass through";
  EXPECT_TRUE(std::isinf(out.ranges[240]))   << "Too-far ray should be filtered";

  removeMerger(merger);
}

// LiDAR tilted 45 deg upward; a ray at 0 deg range=2 m reaches z ~1.41 m > max_height 0.5 m.
TEST_F(ScanMergerTest, SingleLidar_HeightFilter_HighPointsDropped)
{
  auto merger = makeMerger({"/t7/scan"}, "base_link", 0.5, 0.1, 100.0, -100.0, 0.5);

  // Rotation around Y by -45 deg: qy = sin(-22.5 deg), qw = cos(-22.5 deg)
  const double sy = std::sin(-M_PI / 8.0);
  const double cy = std::cos(-M_PI / 8.0);
  broadcastTF(makeTransform("base_link", "lidar_link", 0, 0, 0, 0, sy, 0, cy));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t7/scan", 10);
  spinUntil([&]{ return pub->get_subscription_count() > 0; });

  auto scan = makeScan("lidar_link", helper_->now());
  scan.ranges[180] = 2.0f;  // after tilt: z ~= 2*sin(45) ~= 1.41 m > max_height
  pub->publish(scan);

  ASSERT_TRUE(spinUntil([&]{ return !received.empty(); })) << "No merged scan received";
  EXPECT_TRUE(std::isinf(received.front().ranges[180]))
    << "Ray above max_height should be filtered";

  removeMerger(merger);
}

// NaN and +inf input ranges must be silently skipped; the one finite ray must appear.
TEST_F(ScanMergerTest, SingleLidar_NanInfRanges_Skipped)
{
  auto merger = makeMerger({"/t8/scan"});
  broadcastTF(makeTransform("base_link", "lidar_link", 0, 0, 0, 0, 0, 0, 1));

  std::vector<sensor_msgs::msg::LaserScan> received;
  auto sub = helper_->create_subscription<sensor_msgs::msg::LaserScan>(
    merged_topic_, 10,
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { received.push_back(*msg); });

  auto pub = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/t8/scan", 10);
  spinUntil([&]{ return pub->get_subscription_count() > 0; });

  auto scan = makeScan("lidar_link", helper_->now());
  scan.ranges[100] = std::numeric_limits<float>::quiet_NaN();
  scan.ranges[150] = std::numeric_limits<float>::infinity();
  scan.ranges[180] = 3.0f;
  pub->publish(scan);

  ASSERT_TRUE(spinUntil([&]{ return !received.empty(); })) << "No merged scan received";

  const auto& out = received.front();
  EXPECT_TRUE(std::isinf(out.ranges[100])) << "NaN input should become inf in output";
  EXPECT_TRUE(std::isinf(out.ranges[150])) << "Inf input should remain inf";
  EXPECT_NEAR(out.ranges[180], 3.0f, 0.02f) << "Valid ray should pass through";

  removeMerger(merger);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
