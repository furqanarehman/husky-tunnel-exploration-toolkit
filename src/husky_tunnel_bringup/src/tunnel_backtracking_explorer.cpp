#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TunnelBacktrackingExplorer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  TunnelBacktrackingExplorer()
  : Node("tunnel_backtracking_explorer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_topic_ = declare_parameter("map_topic", std::string("/map"));
    costmap_topic_ = declare_parameter(
      "costmap_topic", std::string("/global_costmap/costmap"));
    global_frame_ = declare_parameter("global_frame", std::string("map"));
    robot_frame_ = declare_parameter("robot_base_frame", std::string("base_link"));
    action_name_ = declare_parameter(
      "navigate_to_pose_action_name", std::string("/navigate_to_pose"));

    planning_period_s_ = declare_parameter("planning_period_s", 2.0);
    no_progress_timeout_s_ = declare_parameter("no_progress_timeout_s", 15.0);
    progress_epsilon_m_ = declare_parameter("progress_epsilon_m", 0.10);
    min_frontier_cells_ = declare_parameter("min_frontier_cells", 8);
    min_goal_distance_m_ = declare_parameter("min_goal_distance_m", 1.0);
    goal_standoff_m_ = declare_parameter("goal_standoff_m", 0.8);
    free_threshold_ = declare_parameter("free_threshold", 20);
    maximum_cost_ = declare_parameter("maximum_cost", 49);
    blacklist_radius_m_ = declare_parameter("blacklist_radius_m", 1.5);
    blacklist_timeout_s_ = declare_parameter("blacklist_timeout_s", 120.0);
    visited_timeout_s_ = declare_parameter("visited_timeout_s", 20.0);
    path_distance_weight_ = declare_parameter("path_distance_weight", 0.35);
    information_gain_weight_ = declare_parameter("information_gain_weight", 1.5);
    completion_confirmations_ = declare_parameter("completion_confirmations", 5);
    return_to_start_on_complete_ =
      declare_parameter("return_to_start_on_complete", true);
    // Hard cap on total mission time. On a large map, coverage-based
    // completion (no_frontier_cycles_ reaching completion_confirmations_)
    // can legitimately take a very long time, or never quite happen if
    // small leftover unvisited patches keep turning up. This forces a
    // return home once the cap is hit, trading full-map completeness for
    // a guaranteed, bounded runtime. Set to 0 to disable (old behavior).
    max_mission_duration_s_ =
      declare_parameter("max_mission_duration_s", 1800.0);

    coverage_enabled_ = declare_parameter("coverage_enabled", true);
    coverage_resolution_m_ = declare_parameter("coverage_resolution_m", 0.5);
    coverage_radius_m_ = declare_parameter("coverage_radius_m", 2.0);
    coverage_min_goal_distance_m_ =
      declare_parameter("coverage_min_goal_distance_m", 2.0);
    coverage_goal_lookahead_m_ =
      declare_parameter("coverage_goal_lookahead_m", 8.0);
    coverage_min_region_cells_ =
      declare_parameter("coverage_min_region_cells", 10);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_qos.reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        map_ = std::move(msg);
      });

    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    costmap_qos.reliable().durability_volatile();
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        costmap_ = std::move(msg);
      });

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(planning_period_s_),
      std::bind(&TunnelBacktrackingExplorer::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Map-driven tunnel backtracking started: map=%s, costmap=%s, action=%s",
      map_topic_.c_str(), costmap_topic_.c_str(), action_name_.c_str());
  }

private:
  struct Cell
  {
    int x{0};
    int y{0};
  };

  struct BlockedRegion
  {
    double x{0.0};
    double y{0.0};
    rclcpp::Time expires;
  };

  struct Candidate
  {
    bool valid{false};
    bool coverage_fallback{false};
    Cell frontier;
    Cell dispatch;
    double frontier_x{0.0};
    double frontier_y{0.0};
    double dispatch_x{0.0};
    double dispatch_y{0.0};
    double path_distance{0.0};
    std::size_t cluster_size{0};
    double score{-std::numeric_limits<double>::infinity()};
  };

  static int index_of(int x, int y, int width)
  {
    return y * width + x;
  }

  static bool inside(int x, int y, int width, int height)
  {
    return x >= 0 && y >= 0 && x < width && y < height;
  }

  static double yaw_to(double from_x, double from_y, double to_x, double to_y)
  {
    return std::atan2(to_y - from_y, to_x - from_x);
  }

  static void set_yaw(geometry_msgs::msg::Pose & pose, double yaw)
  {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw * 0.5);
    pose.orientation.w = std::cos(yaw * 0.5);
  }

  bool world_to_cell(
    const nav_msgs::msg::OccupancyGrid & grid, double wx, double wy, Cell & cell) const
  {
    const double resolution = grid.info.resolution;
    if (resolution <= 0.0) {
      return false;
    }
    cell.x = static_cast<int>(std::floor((wx - grid.info.origin.position.x) / resolution));
    cell.y = static_cast<int>(std::floor((wy - grid.info.origin.position.y) / resolution));
    return inside(
      cell.x, cell.y, static_cast<int>(grid.info.width),
      static_cast<int>(grid.info.height));
  }

  std::pair<double, double> cell_to_world(
    const nav_msgs::msg::OccupancyGrid & grid, const Cell & cell) const
  {
    return {
      grid.info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * grid.info.resolution,
      grid.info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * grid.info.resolution};
  }

  bool traversable(
    const nav_msgs::msg::OccupancyGrid & map,
    const nav_msgs::msg::OccupancyGrid & costmap,
    int x, int y) const
  {
    const int map_width = static_cast<int>(map.info.width);
    const int map_height = static_cast<int>(map.info.height);
    if (!inside(x, y, map_width, map_height)) {
      return false;
    }

    const int8_t map_value = map.data[index_of(x, y, map_width)];
    if (map_value < 0 || map_value > free_threshold_) {
      return false;
    }

    const auto world = cell_to_world(map, Cell{x, y});
    Cell cost_cell;
    if (!world_to_cell(costmap, world.first, world.second, cost_cell)) {
      return false;
    }
    const int cost_width = static_cast<int>(costmap.info.width);
    const int8_t raw_cost = costmap.data[index_of(cost_cell.x, cost_cell.y, cost_width)];
    const int cost = raw_cost < 0 ? 255 : static_cast<int>(raw_cost);
    return cost <= maximum_cost_;
  }

  bool frontier_cell(const nav_msgs::msg::OccupancyGrid & map, int x, int y) const
  {
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    if (!inside(x, y, width, height)) {
      return false;
    }
    const int8_t value = map.data[index_of(x, y, width)];
    if (value < 0 || value > free_threshold_) {
      return false;
    }

    static constexpr int offsets[8][2] = {
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    for (const auto & offset : offsets) {
      const int nx = x + offset[0];
      const int ny = y + offset[1];
      if (inside(nx, ny, width, height) &&
        map.data[index_of(nx, ny, width)] < 0)
      {
        return true;
      }
    }
    return false;
  }

  bool blacklisted(double x, double y)
  {
    const auto now = get_clock()->now();
    blocked_.erase(
      std::remove_if(
        blocked_.begin(), blocked_.end(),
        [&now](const BlockedRegion & region) {return region.expires <= now;}),
      blocked_.end());

    for (const auto & region : blocked_) {
      if (std::hypot(x - region.x, y - region.y) <= blacklist_radius_m_) {
        return true;
      }
    }
    return false;
  }

  void add_blacklist(double x, double y, double duration_s)
  {
    blocked_.push_back(
      BlockedRegion{x, y, get_clock()->now() + rclcpp::Duration::from_seconds(duration_s)});
  }

  static std::uint64_t coverage_key(std::int32_t x, std::int32_t y)
  {
    return
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
      static_cast<std::uint32_t>(y);
  }

  bool coverage_visited(double world_x, double world_y) const
  {
    if (coverage_resolution_m_ <= 0.0) {
      return false;
    }

    const auto x = static_cast<std::int32_t>(
      std::floor(world_x / coverage_resolution_m_));
    const auto y = static_cast<std::int32_t>(
      std::floor(world_y / coverage_resolution_m_));

    return covered_cells_.count(coverage_key(x, y)) > 0;
  }

  void record_coverage(double world_x, double world_y)
  {
    if (!coverage_enabled_ || coverage_resolution_m_ <= 0.0) {
      return;
    }

    if (has_last_coverage_pose_ &&
      std::hypot(
        world_x - last_coverage_x_,
        world_y - last_coverage_y_) < coverage_resolution_m_ * 0.5)
    {
      return;
    }

    last_coverage_x_ = world_x;
    last_coverage_y_ = world_y;
    has_last_coverage_pose_ = true;

    const auto center_x = static_cast<std::int32_t>(
      std::floor(world_x / coverage_resolution_m_));
    const auto center_y = static_cast<std::int32_t>(
      std::floor(world_y / coverage_resolution_m_));

    const int radius = std::max(
      0, static_cast<int>(
        std::ceil(coverage_radius_m_ / coverage_resolution_m_)));

    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::hypot(
            static_cast<double>(dx),
            static_cast<double>(dy)) * coverage_resolution_m_ >
          coverage_radius_m_)
        {
          continue;
        }

        covered_cells_.insert(
          coverage_key(center_x + dx, center_y + dy));
      }
    }
  }

  Candidate select_candidate(
    const nav_msgs::msg::OccupancyGrid & map,
    const nav_msgs::msg::OccupancyGrid & costmap,
    double robot_x, double robot_y)
  {
    Candidate best;
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const int cell_count = width * height;

    Cell start;
    if (!world_to_cell(map, robot_x, robot_y, start)) {
      RCLCPP_WARN(get_logger(), "Robot is outside the current occupancy grid");
      return best;
    }

    std::vector<int> distance(cell_count, -1);
    std::vector<int> parent(cell_count, -1);
    std::queue<Cell> bfs;
    const int start_index = index_of(start.x, start.y, width);
    distance[start_index] = 0;
    bfs.push(start);

    static constexpr int cardinal[4][2] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    while (!bfs.empty()) {
      const Cell current = bfs.front();
      bfs.pop();
      const int current_index = index_of(current.x, current.y, width);
      for (const auto & offset : cardinal) {
        const int nx = current.x + offset[0];
        const int ny = current.y + offset[1];
        if (!inside(nx, ny, width, height) || !traversable(map, costmap, nx, ny)) {
          continue;
        }
        const int next_index = index_of(nx, ny, width);
        if (distance[next_index] >= 0) {
          continue;
        }
        distance[next_index] = distance[current_index] + 1;
        parent[next_index] = current_index;
        bfs.push(Cell{nx, ny});
      }
    }

    std::vector<uint8_t> is_frontier(cell_count, 0);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int idx = index_of(x, y, width);
        if (distance[idx] >= 0 && frontier_cell(map, x, y)) {
          is_frontier[idx] = 1;
        }
      }
    }

    std::vector<uint8_t> visited(cell_count, 0);
    static constexpr int neighbors[8][2] = {
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}};

    for (int seed = 0; seed < cell_count; ++seed) {
      if (!is_frontier[seed] || visited[seed]) {
        continue;
      }
      std::vector<Cell> cluster;
      std::queue<Cell> frontier_queue;
      Cell seed_cell{seed % width, seed / width};
      frontier_queue.push(seed_cell);
      visited[seed] = 1;

      while (!frontier_queue.empty()) {
        const Cell current = frontier_queue.front();
        frontier_queue.pop();
        cluster.push_back(current);
        for (const auto & offset : neighbors) {
          const int nx = current.x + offset[0];
          const int ny = current.y + offset[1];
          if (!inside(nx, ny, width, height)) {
            continue;
          }
          const int idx = index_of(nx, ny, width);
          if (is_frontier[idx] && !visited[idx]) {
            visited[idx] = 1;
            frontier_queue.push(Cell{nx, ny});
          }
        }
      }

      if (static_cast<int>(cluster.size()) < min_frontier_cells_) {
        continue;
      }

      // Select the cell nearest the center of the frontier opening.
      // This avoids choosing an edge cell close to a tunnel wall.
      double center_x = 0.0;
      double center_y = 0.0;
      for (const Cell & cell : cluster) {
        center_x += static_cast<double>(cell.x);
        center_y += static_cast<double>(cell.y);
      }
      center_x /= static_cast<double>(cluster.size());
      center_y /= static_cast<double>(cluster.size());

      Cell frontier = cluster.front();
      int frontier_distance = -1;
      double nearest_to_center =
        std::numeric_limits<double>::infinity();

      for (const Cell & cell : cluster) {
        const int d = distance[index_of(cell.x, cell.y, width)];
        if (d < 0) {
          continue;
        }

        const double center_distance = std::hypot(
          static_cast<double>(cell.x) - center_x,
          static_cast<double>(cell.y) - center_y);

        if (center_distance < nearest_to_center) {
          nearest_to_center = center_distance;
          frontier_distance = d;
          frontier = cell;
        }
      }

      if (frontier_distance < 0) {
        continue;
      }

      const double path_distance = frontier_distance * map.info.resolution;
      if (path_distance < min_goal_distance_m_) {
        continue;
      }
      const auto frontier_world = cell_to_world(map, frontier);
      if (blacklisted(frontier_world.first, frontier_world.second)) {
        continue;
      }

      Cell dispatch = frontier;
      int dispatch_index = index_of(dispatch.x, dispatch.y, width);
      const int standoff_cells = std::max(
        0, static_cast<int>(std::ceil(goal_standoff_m_ / map.info.resolution)));
      for (int step = 0; step < standoff_cells && parent[dispatch_index] >= 0; ++step) {
        dispatch_index = parent[dispatch_index];
        dispatch = Cell{dispatch_index % width, dispatch_index / width};
      }
      const auto dispatch_world = cell_to_world(map, dispatch);

      const double score =
        -path_distance_weight_ * path_distance +
        information_gain_weight_ * std::log1p(static_cast<double>(cluster.size()));
      if (!best.valid || path_distance < best.path_distance) {
        best.valid = true;
        best.frontier = frontier;
        best.dispatch = dispatch;
        best.frontier_x = frontier_world.first;
        best.frontier_y = frontier_world.second;
        best.dispatch_x = dispatch_world.first;
        best.dispatch_y = dispatch_world.second;
        best.path_distance = path_distance;
        best.cluster_size = cluster.size();
        best.score = score;
      }
    }

    // Frontiers always have priority. Their selection above uses the
    // shortest reachable path.
    if (best.valid || !coverage_enabled_) {
      return best;
    }

    // No frontier remains. Search for reachable, known-free cells that the
    // robot has not physically covered.
    std::vector<std::uint8_t> uncovered(cell_count, 0);
    for (int index = 0; index < cell_count; ++index) {
      if (distance[index] < 0) {
        continue;
      }

      const double path_distance =
        static_cast<double>(distance[index]) * map.info.resolution;
      if (path_distance < coverage_min_goal_distance_m_) {
        continue;
      }

      const Cell cell{index % width, index / width};
      const auto world = cell_to_world(map, cell);

      if (!coverage_visited(world.first, world.second) &&
        !blacklisted(world.first, world.second))
      {
        uncovered[index] = 1;
      }
    }

    std::vector<std::uint8_t> coverage_seen(cell_count, 0);
    double best_region_entry_distance =
      std::numeric_limits<double>::infinity();

    for (int seed = 0; seed < cell_count; ++seed) {
      if (!uncovered[seed] || coverage_seen[seed]) {
        continue;
      }

      std::vector<Cell> region;
      std::queue<Cell> region_queue;
      region_queue.push(Cell{seed % width, seed / width});
      coverage_seen[seed] = 1;

      while (!region_queue.empty()) {
        const Cell current = region_queue.front();
        region_queue.pop();
        region.push_back(current);

        for (const auto & offset : cardinal) {
          const int nx = current.x + offset[0];
          const int ny = current.y + offset[1];

          if (!inside(nx, ny, width, height)) {
            continue;
          }

          const int next_index = index_of(nx, ny, width);
          if (uncovered[next_index] && !coverage_seen[next_index]) {
            coverage_seen[next_index] = 1;
            region_queue.push(Cell{nx, ny});
          }
        }
      }

      if (static_cast<int>(region.size()) < coverage_min_region_cells_) {
        continue;
      }

      int nearest_steps = std::numeric_limits<int>::max();
      for (const Cell & cell : region) {
        nearest_steps = std::min(
          nearest_steps,
          distance[index_of(cell.x, cell.y, width)]);
      }

      const int lookahead_cells = std::max(
        1, static_cast<int>(
          std::ceil(coverage_goal_lookahead_m_ / map.info.resolution)));
      const int maximum_target_steps = nearest_steps + lookahead_cells;

      Cell target = region.front();
      int target_steps = -1;

      for (const Cell & cell : region) {
        const int steps = distance[index_of(cell.x, cell.y, width)];
        if (steps <= maximum_target_steps && steps > target_steps) {
          target = cell;
          target_steps = steps;
        }
      }

      const double region_entry_distance =
        static_cast<double>(nearest_steps) * map.info.resolution;

      if (region_entry_distance >= best_region_entry_distance) {
        continue;
      }

      const auto target_world = cell_to_world(map, target);

      best.valid = true;
      best.coverage_fallback = true;
      best.frontier = target;
      best.dispatch = target;
      best.frontier_x = target_world.first;
      best.frontier_y = target_world.second;
      best.dispatch_x = target_world.first;
      best.dispatch_y = target_world.second;
      best.path_distance =
        static_cast<double>(target_steps) * map.info.resolution;
      best.cluster_size = region.size();
      best.score = -region_entry_distance;

      best_region_entry_distance = region_entry_distance;
    }

    return best;
  }

  void tick()
  {
    if (mission_finished_) {
      return;
    }

    if (!mission_start_time_recorded_) {
      mission_start_time_ = get_clock()->now();
      mission_start_time_recorded_ = true;
    }

    if (max_mission_duration_s_ > 0.0 && !mission_deadline_exceeded_) {
      const double mission_elapsed_s =
        (get_clock()->now() - mission_start_time_).seconds();
      if (mission_elapsed_s >= max_mission_duration_s_) {
        mission_deadline_exceeded_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Max mission duration of %.0f s reached; abandoning further "
          "exploration and returning home",
          max_mission_duration_s_);
      }
    }

    if (navigating_) {
      if (goal_handle_ && last_progress_time_.nanoseconds() > 0) {
        const double stalled_for = (get_clock()->now() - last_progress_time_).seconds();
        if (no_progress_timeout_s_ > 0.0 && stalled_for >= no_progress_timeout_s_ && !cancel_requested_) {
          cancel_requested_ = true;
          add_blacklist(active_frontier_x_, active_frontier_y_, blacklist_timeout_s_);
          RCLCPP_WARN(
            get_logger(),
            "No progress for %.1f s at a dead end; canceling and checking the full map",
            stalled_for);
          nav_client_->async_cancel_goal(goal_handle_);
        }
      }
      return;
    }

    if (mission_deadline_exceeded_) {
      // Never fall through to normal frontier/coverage exploration once
      // the deadline has been hit -- only retry getting home, on a
      // cooldown, so a rejected/canceled attempt doesn't immediately
      // fight with a competing exploration goal.
      const double since_last_attempt_s =
        (get_clock()->now() - last_duration_cap_attempt_).seconds();
      if (since_last_attempt_s >= duration_cap_retry_cooldown_s_) {
        last_duration_cap_attempt_ = get_clock()->now();
        send_home_goal();
      }
      return;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map = map_;
      costmap = costmap_;
    }
    if (!map || !costmap) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for /map and global costmap");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for robot transform: %s", exception.what());
      return;
    }

    if (!nav_client_->wait_for_action_server(250ms)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for Nav2 NavigateToPose");
      return;
    }

    const double robot_x = transform.transform.translation.x;
    const double robot_y = transform.transform.translation.y;

    if (!home_pose_recorded_) {
      home_pose_.position.x = robot_x;
      home_pose_.position.y = robot_y;
      home_pose_.position.z = 0.0;
      home_pose_.orientation = transform.transform.rotation;
      home_pose_recorded_ = true;
      RCLCPP_INFO(
        get_logger(), "Recorded return-home pose in %s: (%.2f, %.2f)",
        global_frame_.c_str(), robot_x, robot_y);
    }

    record_coverage(robot_x, robot_y);

    Candidate candidate = select_candidate(*map, *costmap, robot_x, robot_y);
    if (!candidate.valid) {
      ++no_frontier_cycles_;
      if (no_frontier_cycles_ >= completion_confirmations_) {
        if (return_to_start_on_complete_) {
          RCLCPP_INFO(
            get_logger(),
            "No reachable frontier or meaningful unvisited region remains after %d checks; "
            "exploration is complete, returning home",
            completion_confirmations_);
          send_home_goal();
        } else {
          mission_finished_ = true;
          RCLCPP_INFO(
            get_logger(),
            "No reachable frontier or meaningful unvisited region remains after %d checks; "
            "exploration is complete",
            completion_confirmations_);
        }
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No reachable frontier or meaningful unvisited region currently available; "
          "checking the map again");
      }
      return;
    }
    no_frontier_cycles_ = 0;
    send_goal(candidate);
  }

  void send_home_goal()
  {
    if (!home_pose_recorded_) {
      RCLCPP_ERROR(
        get_logger(), "Cannot return home because the initial map pose was not recorded");
      mission_finished_ = true;
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = get_clock()->now();
    goal.pose.pose = home_pose_;

    best_distance_remaining_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = get_clock()->now();
    cancel_requested_ = false;
    returning_home_ = true;
    navigating_ = true;

    RCLCPP_INFO(
      get_logger(), "Returning to recorded home pose: (%.2f, %.2f)",
      home_pose_.position.x, home_pose_.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr handle) {
        if (!handle) {
          RCLCPP_WARN(get_logger(), "Nav2 rejected the return-home goal; retrying later");
          returning_home_ = false;
          navigating_ = false;
          no_frontier_cycles_ = 0;
          return;
        }
        goal_handle_ = handle;
      };
    options.feedback_callback =
      [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        const double remaining = feedback->distance_remaining;
        if (!std::isfinite(best_distance_remaining_) ||
          remaining <= best_distance_remaining_ - progress_epsilon_m_)
        {
          best_distance_remaining_ = remaining;
          last_progress_time_ = get_clock()->now();
        }
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        goal_handle_.reset();
        navigating_ = false;
        cancel_requested_ = false;
        returning_home_ = false;

        if (succeeded) {
          mission_finished_ = true;
          RCLCPP_INFO(get_logger(), "Home pose reached; autonomous mission finished");
        } else {
          no_frontier_cycles_ = 0;
          RCLCPP_WARN(
            get_logger(),
            "Return-home navigation ended with result code %d; retrying after map checks",
            static_cast<int>(result.code));
        }
      };
    nav_client_->async_send_goal(goal, options);
  }

  void send_goal(const Candidate & candidate)
  {
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = get_clock()->now();
    goal.pose.pose.position.x = candidate.dispatch_x;
    goal.pose.pose.position.y = candidate.dispatch_y;
    goal.pose.pose.position.z = 0.0;
    set_yaw(
      goal.pose.pose,
      yaw_to(
        candidate.dispatch_x, candidate.dispatch_y,
        candidate.frontier_x, candidate.frontier_y));

    active_frontier_x_ = candidate.frontier_x;
    active_frontier_y_ = candidate.frontier_y;
    best_distance_remaining_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = get_clock()->now();
    cancel_requested_ = false;
    navigating_ = true;

    if (candidate.coverage_fallback) {
      RCLCPP_INFO(
        get_logger(),
        "No frontier remains; navigating to nearest unvisited region: "
        "goal=(%.2f, %.2f), path=%.2f m, cells=%zu",
        candidate.dispatch_x, candidate.dispatch_y,
        candidate.path_distance, candidate.cluster_size);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Navigating to nearest reachable frontier: "
        "goal=(%.2f, %.2f), path=%.2f m, cells=%zu",
        candidate.dispatch_x, candidate.dispatch_y,
        candidate.path_distance, candidate.cluster_size);
    }

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr handle) {
        if (!handle) {
          RCLCPP_WARN(get_logger(), "Nav2 rejected the frontier goal");
          add_blacklist(active_frontier_x_, active_frontier_y_, blacklist_timeout_s_);
          navigating_ = false;
          return;
        }
        goal_handle_ = handle;
      };
    options.feedback_callback =
      [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        record_coverage(
          feedback->current_pose.pose.position.x,
          feedback->current_pose.pose.position.y);

        const double remaining = feedback->distance_remaining;
        if (!std::isfinite(best_distance_remaining_) ||
          remaining <= best_distance_remaining_ - progress_epsilon_m_)
        {
          best_distance_remaining_ = remaining;
          last_progress_time_ = get_clock()->now();
        }
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        if (succeeded) {
          RCLCPP_INFO(
            get_logger(), "Frontier reached; scanning the complete map for the next branch");
          add_blacklist(active_frontier_x_, active_frontier_y_, visited_timeout_s_);
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Frontier navigation ended with result code %d; selecting another reachable branch",
            static_cast<int>(result.code));
          add_blacklist(active_frontier_x_, active_frontier_y_, blacklist_timeout_s_);
        }
        goal_handle_.reset();
        navigating_ = false;
        cancel_requested_ = false;
      };
    nav_client_->async_send_goal(goal, options);
  }

  std::string map_topic_;
  std::string costmap_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string action_name_;
  double planning_period_s_{2.0};
  double no_progress_timeout_s_{15.0};
  double progress_epsilon_m_{0.10};
  int min_frontier_cells_{8};
  double min_goal_distance_m_{1.0};
  double goal_standoff_m_{0.8};
  int free_threshold_{20};
  int maximum_cost_{49};
  double blacklist_radius_m_{1.5};
  double blacklist_timeout_s_{120.0};
  double visited_timeout_s_{20.0};
  double path_distance_weight_{0.35};
  double information_gain_weight_{1.5};
  int completion_confirmations_{5};
  bool return_to_start_on_complete_{true};

  bool coverage_enabled_{true};
  double coverage_resolution_m_{0.5};
  double coverage_radius_m_{2.0};
  double coverage_min_goal_distance_m_{2.0};
  double coverage_goal_lookahead_m_{8.0};
  int coverage_min_region_cells_{10};

  std::unordered_set<std::uint64_t> covered_cells_;
  bool has_last_coverage_pose_{false};
  double last_coverage_x_{0.0};
  double last_coverage_y_{0.0};

  std::mutex data_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  std::vector<BlockedRegion> blocked_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool navigating_{false};
  bool cancel_requested_{false};
  bool returning_home_{false};
  bool mission_finished_{false};
  double max_mission_duration_s_{1800.0};
  bool mission_start_time_recorded_{false};
  rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};
  // Permanent one-way switch: once the mission duration cap is hit, this
  // stays true for the rest of the run. tick() checks this BEFORE ever
  // reaching select_candidate()/send_goal(), so normal frontier/coverage
  // exploration can never send a competing goal again -- previously,
  // returning_home_ alone was used as the gate, but that gets reset to
  // false by send_home_goal's own result callback on any non-success
  // result (including being canceled/preempted), which let normal
  // exploration slip back in and immediately preempt the next home
  // attempt, causing an unbounded tug-of-war between the two goals.
  bool mission_deadline_exceeded_{false};
  rclcpp::Time last_duration_cap_attempt_{0, 0, RCL_ROS_TIME};
  double duration_cap_retry_cooldown_s_{20.0};
  bool home_pose_recorded_{false};
  geometry_msgs::msg::Pose home_pose_;
  int no_frontier_cycles_{0};
  double active_frontier_x_{0.0};
  double active_frontier_y_{0.0};
  double best_distance_remaining_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TunnelBacktrackingExplorer>());
  rclcpp::shutdown();
  return 0;
}
