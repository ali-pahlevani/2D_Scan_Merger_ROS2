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
 * @file scan_2d_merger.cpp
 * @author Ali Pahlevani (a.pahlevani2050@gmail.com)
 * @brief Laser scan merger for 1 to N 2D LiDARs
 * @version 2.0.0
 * @date 2026-06-03
 */
#include "scan_2d_merger/scan_2d_merger.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>

RCLCPP_COMPONENTS_REGISTER_NODE(util::LaserScanMerger)

namespace util
{

LaserScanMerger::LaserScanMerger(const rclcpp::NodeOptions& opts)
  : rclcpp::Node{"component_scan_2d_merger", opts}
  , tf_buffer_{std::make_unique<tf2_ros::Buffer>(get_clock())}
  , tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)}
{
  loadParams();
  if (validateParams()) {
    start();
  } else {
    RCLCPP_ERROR(get_logger(), "%s: startup aborted - fix parameters and restart.", get_name());
  }
}

LaserScanMerger::~LaserScanMerger()
{
  if (scan_workers_.empty()) return;

  // Set the shutdown flag before releasing the barrier so workers exit their loops.
  workers_shutdown_.store(true, std::memory_order_relaxed);
  start_barrier_->arrive_and_wait();
  for (auto& w : scan_workers_) w.join();
}

void LaserScanMerger::loadParams()
{
  scan_topics_     = declare_parameter<std::vector<std::string>>("scan_topics",    std::vector<std::string>{});
  scan_policies_   = declare_parameter<std::vector<int64_t>>("scan_policies",      std::vector<int64_t>{});
  merged_frame_id_ = declare_parameter<std::string>("merged_frame_id", "base_link");
  output_topic_    = declare_parameter<std::string>("output_topic",    "merged_scan");
  debug_           = declare_parameter<bool>("debug",                  false);
  moving_frames_   = declare_parameter<bool>("moving_frames",          false);
  queue_size_      = declare_parameter<int>("queue_size",              20);
  sync_slop_       = declare_parameter<double>("sync_slop",            0.1);
  ang_min_         = declare_parameter<double>("angle_min",            -M_PI);
  ang_max_         = declare_parameter<double>("angle_max",             M_PI);
  ang_increment_   = declare_parameter<double>("angle_increment",       M_PI / 180.0);
  range_min_       = declare_parameter<double>("range_min",            0.1);
  range_max_       = declare_parameter<double>("range_max",            static_cast<double>(std::numeric_limits<float>::max()));
  inf_eps_         = declare_parameter<double>("inf_epsilon",          1.0);
  use_inf_         = declare_parameter<bool>("use_inf",                true);
  scan_time_       = declare_parameter<double>("scan_time",            1.0 / 30.0);
  min_height_      = declare_parameter<double>("min_height",           std::numeric_limits<double>::lowest());
  max_height_      = declare_parameter<double>("max_height",           std::numeric_limits<double>::max());
  tolerance_       = declare_parameter<double>("tolerance",            0.01);
}

bool LaserScanMerger::validateParams() const
{
  if (scan_topics_.empty()) {
    RCLCPP_ERROR(get_logger(), "scan_topics must not be empty.");
    return false;
  }
  if (ang_min_ >= ang_max_) {
    RCLCPP_ERROR(get_logger(), "angle_min (%.4f) must be less than angle_max (%.4f).", ang_min_, ang_max_);
    return false;
  }
  if (ang_increment_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "angle_increment must be positive (got %.6f).", ang_increment_);
    return false;
  }
  if (range_min_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "range_min must be >= 0 (got %.4f).", range_min_);
    return false;
  }
  if (range_min_ >= range_max_) {
    RCLCPP_ERROR(get_logger(), "range_min (%.4f) must be less than range_max (%.4f).", range_min_, range_max_);
    return false;
  }
  if (queue_size_ <= 0) {
    RCLCPP_ERROR(get_logger(), "queue_size must be positive (got %d).", queue_size_);
    return false;
  }
  if (sync_slop_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "sync_slop must be >= 0 (got %.4f).", sync_slop_);
    return false;
  }
  return true;
}

void LaserScanMerger::start()
{
  const std::size_t n = scan_topics_.size();

  if (scan_policies_.size() < n) {
    if (!scan_policies_.empty())
      RCLCPP_WARN(get_logger(),
        "scan_policies has %zu entries for %zu topics - filling missing entries with 'reliable'.",
        scan_policies_.size(), n);
    scan_policies_.resize(n, 0);
  }

  ranges_size_ = static_cast<uint32_t>(std::ceil((ang_max_ - ang_min_) / ang_increment_));
  scan_buffers_.resize(n);

  // MutuallyExclusive ensures only one scanCallback runs at a time, even with a
  // multi-threaded executor, so shared state needs no additional locking beyond buffers_mutex_.
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  for (std::size_t i = 0; i < n; ++i) {
    rclcpp::QoS qos(queue_size_);
    scan_policies_[i] == 0 ? qos.reliable() : qos.best_effort();

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group_;

    scan_subs_.push_back(
      create_subscription<LaserScanMsg>(
        scan_topics_[i], qos,
        [this, i](LaserScanPtr msg) { scanCallback(std::move(msg), i); },
        sub_opts
      )
    );

    if (debug_)
      RCLCPP_INFO(get_logger(), "Subscribed to '%s' (%s QoS).",
        scan_topics_[i].c_str(), scan_policies_[i] == 0 ? "reliable" : "best-effort");
  }

  merged_pub_ = create_publisher<LaserScanMsg>(output_topic_, rclcpp::QoS(1));
  transforms_.reserve(n);

  // For N > 1, spawn one persistent worker per LiDAR; barriers are N+1 to include
  // the main thread. For N = 1, projection runs directly on the main thread.
  if (n > 1) {
    worker_scan_ptrs_.resize(n, nullptr);
    worker_ranges_.resize(n);
    for (auto& r : worker_ranges_) r.resize(ranges_size_);

    start_barrier_ = std::make_unique<std::barrier<>>(static_cast<std::ptrdiff_t>(n + 1));
    done_barrier_  = std::make_unique<std::barrier<>>(static_cast<std::ptrdiff_t>(n + 1));

    for (std::size_t i = 0; i < n; ++i)
      scan_workers_.emplace_back(&LaserScanMerger::workerLoop, this, i);
  }

  RCLCPP_INFO(get_logger(), "%s: merging %zu scan(s) -> '%s'  [%u bins, %.4f to %.4f rad].",
    get_name(), n, output_topic_.c_str(), ranges_size_, ang_min_, ang_max_);
}

void LaserScanMerger::scanCallback(LaserScanPtr msg, std::size_t idx)
{
  // Hold the lock only for buffering and sync; release before mergeAndPublish so
  // the parallel projection does not delay the next incoming scan.
  std::vector<LaserScanPtr> synced;
  {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    scan_buffers_[idx].push_back(std::move(msg));
    while (static_cast<int>(scan_buffers_[idx].size()) > queue_size_)
      scan_buffers_[idx].pop_front();
    trySync(synced);
  }
  if (!synced.empty())
    mergeAndPublish(synced);
}

bool LaserScanMerger::trySync(std::vector<LaserScanPtr>& result)
{
  const std::size_t n = scan_buffers_.size();

  // Single-lidar path: no cross-topic synchronisation needed.
  if (n == 1) {
    if (scan_buffers_[0].empty()) return false;
    result = {scan_buffers_[0].front()};
    scan_buffers_[0].pop_front();
    return true;
  }

  for (const auto& buf : scan_buffers_)
    if (buf.empty()) return false;

  // Use the newest available stamp across all topics as the sync reference point.
  int64_t t_ref_ns = 0;
  for (const auto& buf : scan_buffers_) {
    const auto& s = buf.back()->header.stamp;
    const int64_t t = static_cast<int64_t>(s.sec) * 1'000'000'000LL + s.nanosec;
    if (t > t_ref_ns) t_ref_ns = t;
  }

  // Pass 1: find the closest scan per topic without consuming anything.
  // If any topic falls outside sync_slop, return early with all buffers intact.
  using Iter = std::deque<LaserScanPtr>::iterator;
  std::vector<Iter> best_its(n);
  for (std::size_t i = 0; i < n; ++i) {
    double best_dt = std::numeric_limits<double>::max();
    Iter   best_it = scan_buffers_[i].begin();

    for (auto it = scan_buffers_[i].begin(); it != scan_buffers_[i].end(); ++it) {
      const auto& s = (*it)->header.stamp;
      const int64_t t = static_cast<int64_t>(s.sec) * 1'000'000'000LL + s.nanosec;
      const double dt = std::abs(static_cast<double>(t - t_ref_ns) * 1e-9);
      if (dt < best_dt) { best_dt = dt; best_it = it; }
    }

    if (best_dt > sync_slop_) return false;
    best_its[i] = best_it;
  }

  // Pass 2: all topics matched — dequeue the selected scans.
  result.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    result[i] = *best_its[i];
    scan_buffers_[i].erase(scan_buffers_[i].begin(), std::next(best_its[i]));
  }
  return true;
}

bool LaserScanMerger::lookupTransforms(const std::vector<LaserScanPtr>& scans)
{
  // When frames are static, look up once and reuse; moving_frames forces a refresh each cycle.
  if (!moving_frames_ && transforms_ready_) return true;

  std::vector<tf2::Transform> new_transforms;
  new_transforms.reserve(scans.size());

  for (const auto& scan : scans) {
    try {
      const auto ts = tf_buffer_->lookupTransform(
        merged_frame_id_,
        scan->header.frame_id,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(tolerance_)
      );
      tf2::Transform tf;
      tf2::fromMsg(ts.transform, tf);
      new_transforms.push_back(tf);
    } catch (const tf2::TransformException& e) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed for frame '%s': %s",
        scan->header.frame_id.c_str(), e.what());
      return false;
    }
  }

  transforms_ = std::move(new_transforms);
  transforms_ready_ = true;
  return true;
}

void LaserScanMerger::projectScan(
  const LaserScanMsg& scan,
  const tf2::Transform& tf,
  std::vector<float>& out_ranges) const
{
  for (std::size_t ray = 0; ray < scan.ranges.size(); ++ray) {
    const float r = scan.ranges[ray];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

    const double angle = scan.angle_min + static_cast<double>(ray) * scan.angle_increment;
    const tf2::Vector3 pt = tf * tf2::Vector3(r * std::cos(angle), r * std::sin(angle), 0.0);

    if (pt.z() < min_height_ || pt.z() > max_height_) continue;

    const double range_2d = std::hypot(pt.x(), pt.y());
    if (range_2d < range_min_ || range_2d > range_max_) continue;

    const double merged_angle = std::atan2(pt.y(), pt.x());
    if (merged_angle < ang_min_ || merged_angle > ang_max_) continue;

    // float32 angle fields in the input scan accumulate ~1e-7 rad rounding error per ray.
    // Truncating the bin index can land a reading in the wrong bin; rounding corrects this.
    const int bin = static_cast<int>(std::round((merged_angle - ang_min_) / ang_increment_));
    if (static_cast<uint32_t>(bin) >= ranges_size_) continue;

    const auto r2d = static_cast<float>(range_2d);
    if (r2d < out_ranges[bin]) out_ranges[bin] = r2d;
  }
}

void LaserScanMerger::workerLoop(std::size_t idx)
{
  while (true) {
    start_barrier_->arrive_and_wait();
    if (workers_shutdown_.load(std::memory_order_relaxed)) break;
    projectScan(*worker_scan_ptrs_[idx], transforms_[idx], worker_ranges_[idx]);
    done_barrier_->arrive_and_wait();
  }
}

void LaserScanMerger::mergeAndPublish(const std::vector<LaserScanPtr>& scans)
{
  if (merged_pub_->get_subscription_count() == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "no subscribers on '%s' - skipping publish.", output_topic_.c_str());
    return;
  }

  if (!lookupTransforms(scans)) return;

  const std::size_t n = scans.size();
  const float fill_value = use_inf_
    ? std::numeric_limits<float>::infinity()
    : static_cast<float>(range_max_ + inf_eps_);

  auto out = std::make_unique<LaserScanMsg>();
  out->header.stamp    = scans.front()->header.stamp;
  out->header.frame_id = merged_frame_id_;
  out->angle_min       = static_cast<float>(ang_min_);
  out->angle_max       = static_cast<float>(ang_max_);
  out->angle_increment = static_cast<float>(ang_increment_);
  out->time_increment  = 0.0f;
  out->scan_time       = static_cast<float>(scan_time_);
  out->range_min       = static_cast<float>(range_min_);
  out->range_max       = static_cast<float>(range_max_);

  if (scan_workers_.empty()) {
    // N = 1: project the single scan directly into the output array.
    out->ranges.assign(ranges_size_, fill_value);
    projectScan(*scans[0], transforms_[0], out->ranges);
  } else {
    // N > 1: dispatch to worker threads, then fold their results.
    for (std::size_t i = 0; i < n; ++i) {
      worker_scan_ptrs_[i] = scans[i].get();
      std::fill(worker_ranges_[i].begin(), worker_ranges_[i].end(), fill_value);
    }

    start_barrier_->arrive_and_wait();
    done_barrier_->arrive_and_wait();

    // Combine per-worker arrays by taking the minimum range at each angular bin.
    out->ranges = worker_ranges_[0];
    for (std::size_t i = 1; i < n; ++i) {
      std::transform(
        out->ranges.begin(), out->ranges.end(),
        worker_ranges_[i].begin(),
        out->ranges.begin(),
        [](float a, float b) { return std::min(a, b); }
      );
    }
  }

  merged_pub_->publish(std::move(out));
}

} // namespace util
