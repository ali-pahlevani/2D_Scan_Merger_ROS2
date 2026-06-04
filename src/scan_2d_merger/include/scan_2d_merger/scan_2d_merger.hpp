/**
 * Copyright 2026 Ali Pahlevani
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file scan_2d_merger.hpp
 * @author Ali Pahlevani (a.pahlevani2050@gmail.com)
 * @brief Laser scan merger for 1 to N 2D LiDARs
 * @version 2.0.0
 * @date 2026-06-03
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"

#include <atomic>
#include <barrier>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace util
{

class LaserScanMerger : public rclcpp::Node
{
public:
  using LaserScanMsg = sensor_msgs::msg::LaserScan;
  using LaserScanPtr = LaserScanMsg::ConstSharedPtr;

  explicit LaserScanMerger(const rclcpp::NodeOptions& opts);
  ~LaserScanMerger();

private:
  // TF buffer and listener; transforms_ caches one entry per input scan frame.
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<tf2::Transform>                 transforms_;
  bool                                        transforms_ready_{false};

  // Serialises scanCallback invocations so no two run concurrently, regardless of executor type.
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Per-topic subscribers and FIFO scan buffers; buffers_mutex_ guards all buffer access.
  std::vector<rclcpp::Subscription<LaserScanMsg>::SharedPtr> scan_subs_;
  std::vector<std::deque<LaserScanPtr>>                      scan_buffers_;
  std::mutex                                                 buffers_mutex_;

  rclcpp::Publisher<LaserScanMsg>::SharedPtr merged_pub_;

  uint32_t ranges_size_{0};  // number of output angular bins; fixed after startup

  // One persistent thread per LiDAR (N > 1 only). Threads are reused across merge
  // cycles via std::barrier to avoid per-callback thread-creation overhead.
  std::vector<std::thread>        scan_workers_;
  std::atomic<bool>               workers_shutdown_{false};
  std::unique_ptr<std::barrier<>> start_barrier_;  // main -> workers: begin projection
  std::unique_ptr<std::barrier<>> done_barrier_;   // workers -> main: projection done
  std::vector<const LaserScanMsg*> worker_scan_ptrs_;  // set by main before start_barrier
  std::vector<std::vector<float>>  worker_ranges_;     // each worker writes its own array

  std::vector<std::string> scan_topics_;
  std::vector<int64_t>     scan_policies_;
  std::string  merged_frame_id_;
  std::string  output_topic_;
  bool         debug_{false};
  bool         moving_frames_{false};
  int          queue_size_{20};
  double       sync_slop_{0.1};
  double       ang_min_{-M_PI};
  double       ang_max_{M_PI};
  double       ang_increment_{M_PI / 180.0};
  double       range_min_{0.1};
  double       range_max_{std::numeric_limits<double>::max()};
  double       inf_eps_{1.0};
  bool         use_inf_{true};
  double       scan_time_{1.0 / 30.0};
  double       min_height_{std::numeric_limits<double>::lowest()};
  double       max_height_{std::numeric_limits<double>::max()};
  double       tolerance_{0.01};

  void loadParams();
  bool validateParams() const;
  void start();

  void scanCallback(LaserScanPtr msg, std::size_t idx);
  bool trySync(std::vector<LaserScanPtr>& result);

  bool lookupTransforms(const std::vector<LaserScanPtr>& scans);
  void mergeAndPublish(const std::vector<LaserScanPtr>& scans);

  // Projects rays from one scan into out_ranges, keeping the minimum range per bin.
  // Called from worker threads (N > 1) and directly on the main thread (N = 1).
  void projectScan(const LaserScanMsg& scan, const tf2::Transform& tf,
                   std::vector<float>& out_ranges) const;

  void workerLoop(std::size_t idx);
};

} // namespace util
