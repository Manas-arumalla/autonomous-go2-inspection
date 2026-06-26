// frontier_explorer -- autonomous map building for the Go2.
//
// Reads the SLAM occupancy grid (/map), finds frontiers (free cells adjacent to unknown),
// BFS-clusters them, picks the best navigable frontier by INFORMATION GAIN (large unknown reveal,
// penalised by distance -- explore_lite style, not pure nearest), and drives there via the Nav2
// NavigateToPose action.
//
// RECOVERY IS OWNED BY NAV2: we never drive /cmd_vel ourselves. If the robot gets near a goal but
// can't finish -> advance to the next frontier; if it stops making progress (wedged) -> blacklist +
// replan to a different frontier, letting Nav2's OWN recovery (BackUp/Spin -- which simulate ahead
// and stop before hitting) do any backing-up SAFELY. (A blind /cmd_vel reverse would hit obstacles
// behind the robot.) Blacklist entries expire (TTL) so no area is abandoned forever.
// FSM: IDLE -> EXPLORING -> NAVIGATING -> (EXPLORING | DONE).
//
// Depends only on /map (OccupancyGrid), TF (map->base_link), and Nav2 NavigateToPose -- so it is
// simulator-agnostic and ports unchanged to the real Go2 (ADR-002/007).
#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

struct Frontier { double wx, wy; int size; double dist; double score; };

class FrontierExplorer : public rclcpp::Node {
public:
  FrontierExplorer() : Node("frontier_explorer") {
    min_frontier_size_ = declare_parameter("min_frontier_size", 8);
    min_goal_distance_ = declare_parameter("min_goal_distance", 0.8);  // > Nav2 goal tolerance so each goal demands real travel
    free_thresh_       = declare_parameter("free_threshold", 25);
    blacklist_radius_  = declare_parameter("blacklist_radius", 0.5);
    // A goal must be >= goal_clearance from any WALL, else the waypoint lands in/against a wall (inside the
    // costmap inflation) and Nav2 can't fit the footprint there -> goal rejected/blacklisted, time wasted.
    // Unknown(-1) cells DON'T count (a frontier is next to unknown by definition) -- only occupied(>=thresh).
    // MUST be ~ the global-costmap inflation_radius (0.7) AND > the footprint reach (0.35m half-length): then
    // frontiers are only picked in genuinely FREE space and near-wall ones are OMITTED (the explorer just
    // looks for a freer point / cluster). 0.30 was too small -> goals fell in the inflation -> unreachable.
    goal_clearance_    = declare_parameter("goal_clearance", 0.6);      // ~ inflation_radius -> reachable goals
    occupied_thresh_   = declare_parameter("occupied_threshold", 65);
    goal_timeout_s_    = declare_parameter("goal_timeout", 60.0);
    planning_period_   = declare_parameter("planning_period", 2.0);
    // Reactive goal handling (anti-circling / anti-stuck). Exploration needs no pinpoint reaching --
    // once near a frontier the LiDAR already sees the unknown beyond it, so we advance.
    arrival_radius_    = declare_parameter("arrival_radius", 0.55);   // "close enough" to a frontier
    arrival_patience_  = declare_parameter("arrival_patience", 3.0);  // s lingering near goal => advance
    progress_dist_     = declare_parameter("progress_dist", 0.25);    // m of motion counted as progress
    // s without progress => give up on this goal (blacklist + replan). Long enough that Nav2's own
    // recovery (clear costmap / spin / collision-aware BackUp) gets to run first.
    progress_timeout_  = declare_parameter("progress_timeout", 18.0);
    // INFO-GAIN selection (explore_lite style): prefer LARGE frontiers (more unknown revealed),
    // penalised by distance, over pure nearest -> purposeful "where to go", less dithering.
    gain_scale_        = declare_parameter("gain_scale", 1.0);
    potential_scale_   = declare_parameter("potential_scale", 3.0);
    // Blacklisted frontiers EXPIRE after this -> retried once the map fills in (no permanent give-up).
    blacklist_ttl_     = declare_parameter("blacklist_ttl", 30.0);
    // SHORT-GOAL STEPPING: cap how far a single goal can be -- send a nearer intermediate toward the chosen
    // frontier, then re-evaluate. Short reachable goals => Nav2 rarely times out/wedges => far regions
    // aren't blacklisted+abandoned (the premature-COMPLETE cause); the robot STEPS toward distant frontiers.
    max_goal_distance_ = declare_parameter("max_goal_distance", 3.0);
    // Don't quit the instant frontiers are empty: if frontier CELLS remain but were all blacklisted, clear
    // the blacklist and retry (bounded by max_clear_retries between successes), and require done_confirm
    // consecutive truly-empty cycles -> no premature stop while reachable area still exists.
    done_confirm_      = declare_parameter("done_confirm", 3);
    max_clear_retries_ = declare_parameter("max_clear_retries", 3);
    bool autostart     = declare_parameter("autostart", true);
    robot_frame_       = declare_parameter("robot_base_frame", std::string("base_link"));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(1); map_qos.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos, [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){ map_ = m; });

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/explore/frontiers", 1);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    start_srv_ = create_service<std_srvs::srv::Empty>("/explore/start",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>){ state_ = EXPLORING;
             RCLCPP_INFO(get_logger(), "exploration started"); });
    stop_srv_ = create_service<std_srvs::srv::Empty>("/explore/stop",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>){ state_ = IDLE; cancelGoal();
             RCLCPP_INFO(get_logger(), "exploration stopped"); });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(planning_period_), [this]{ tick(); });
    state_ = autostart ? EXPLORING : IDLE;
    RCLCPP_INFO(get_logger(), "frontier_explorer ready (autostart=%d)", autostart);
  }

private:
  enum State { IDLE, EXPLORING, NAVIGATING, DONE };

  bool robotPose(double &x, double &y) {
    try {
      auto t = tf_buffer_->lookupTransform("map", robot_frame_, tf2::TimePointZero);
      x = t.transform.translation.x; y = t.transform.translation.y; return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no map->%s TF yet: %s",
                           robot_frame_.c_str(), e.what());
      return false;
    }
  }

  // True once SLAM has produced a real map (>= min_cells free cells). Guards against an empty
  // initial /map latching the FSM to DONE before exploration even begins.
  bool mapHasFreeSpace(int min_cells) {
    if (!map_) return false;
    int n = 0;
    for (auto v : map_->data) if (v >= 0 && v <= free_thresh_) if (++n >= min_cells) return true;
    return false;
  }

  void tick() {
    // keep the blacklist bounded to its TTL window (clampGoal + frontier selection scan it every tick).
    if (!blacklist_.empty()) {
      auto nowt = now();
      blacklist_.erase(std::remove_if(blacklist_.begin(), blacklist_.end(),
        [&](const BL &b){ return (nowt - b.t).seconds() >= blacklist_ttl_; }), blacklist_.end());
    }
    if (state_ == NAVIGATING) { watchdog(); return; }   // timeout/recovery checks while driving
    if (state_ != EXPLORING) return;
    if (!map_) { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for /map"); return; }
    double rx, ry; if (!robotPose(rx, ry)) return;

    auto frontiers = findFrontiers(rx, ry);
    publishMarkers(frontiers);
    // Genuine progress = the map GREW (new free cells). Refresh the clear-retry budget only then -- NOT on
    // mere goal-arrival, which fires precisely when the robot is circling a frontier it can't consume
    // (resetting there would defeat max_clear_retries and livelock the FSM short of DONE).
    if (cur_free_cells_ > free_high_water_) { free_high_water_ = cur_free_cells_; clears_done_ = 0; }
    if (frontiers.empty()) {
      if (!mapHasFreeSpace(50)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map not populated yet -- waiting for SLAM");
        return;
      }
      // Frontier CELLS still exist but none were navigable this cycle (all blacklisted / too-far-failed):
      // clear the (TTL) blacklist and retry before giving up, so reachable-but-previously-failed regions
      // aren't abandoned. Bounded by max_clear_retries between successes (reset on a reached frontier).
      if (last_frontier_cells_ > 0 && !blacklist_.empty() && clears_done_ < max_clear_retries_) {
        RCLCPP_INFO(get_logger(), "no navigable frontier (%d cells, all blacklisted/filtered) -- clearing "
                    "blacklist + retrying (%d/%d)", last_frontier_cells_, clears_done_ + 1, max_clear_retries_);
        blacklist_.clear(); ++clears_done_; empty_cycles_ = 0; return;
      }
      if (++empty_cycles_ < done_confirm_) return;     // require a few consecutive empty cycles
      RCLCPP_INFO(get_logger(), "no reachable frontiers left -- exploration COMPLETE");
      state_ = DONE; return;
    }
    empty_cycles_ = 0;
    // INFO-GAIN: highest score first (large frontier, near) rather than pure nearest.
    std::sort(frontiers.begin(), frontiers.end(),
              [](const Frontier &a, const Frontier &b){ return a.score > b.score; });
    double gx = frontiers.front().wx, gy = frontiers.front().wy;
    clampGoal(gx, gy, rx, ry);                          // step toward far frontiers instead of one long goal
    sendGoal(gx, gy, rx, ry);
  }

  std::vector<Frontier> findFrontiers(double rx, double ry) {
    const auto &g = *map_;
    const int W = g.info.width, H = g.info.height;
    const double res = g.info.resolution, ox = g.info.origin.position.x, oy = g.info.origin.position.y;
    auto idx = [&](int cx, int cy){ return cy * W + cx; };
    auto isFree = [&](int v){ return v >= 0 && v <= free_thresh_; };
    // A candidate goal cell is only valid if NO occupied (wall) cell lies within clr cells of it,
    // so Nav2 can fit the robot footprint there (fixes "waypoint inside the wall").
    const int clr = std::max(1, (int)std::lround(goal_clearance_ / res));
    auto hasClearance = [&](int cx, int cy){
      for (int dy = -clr; dy <= clr; ++dy)
        for (int dx = -clr; dx <= clr; ++dx) {
          int nx = cx + dx, ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
          if (g.data[idx(nx, ny)] >= occupied_thresh_) return false;
        }
      return true;
    };

    std::vector<char> isFrontier(W * H, 0);
    for (int cy = 1; cy < H - 1; ++cy)
      for (int cx = 1; cx < W - 1; ++cx) {
        if (!isFree(g.data[idx(cx, cy)])) continue;
        bool nearUnknown = false;
        for (int dy = -1; dy <= 1 && !nearUnknown; ++dy)
          for (int dx = -1; dx <= 1; ++dx)
            if (g.data[idx(cx + dx, cy + dy)] == -1) { nearUnknown = true; break; }
        if (nearUnknown) isFrontier[idx(cx, cy)] = 1;
      }

    std::vector<Frontier> out;
    std::vector<char> visited(W * H, 0);
    for (int cy = 1; cy < H - 1; ++cy)
      for (int cx = 1; cx < W - 1; ++cx) {
        int i = idx(cx, cy);
        if (!isFrontier[i] || visited[i]) continue;
        // BFS cluster. Target the cluster cell NEAREST the robot (>= min_goal_distance, with
        // wall-clearance), not the centroid: a frontier ring around the robot would have its
        // centroid back on the robot, which min_goal_distance then rejects.
        std::deque<int> q{i}; visited[i] = 1;
        int n = 0; double best_d = 1e18, best_wx = 0, best_wy = 0;
        while (!q.empty()) {
          int c = q.front(); q.pop_front();
          int ccx = c % W, ccy = c / W; ++n;
          double cwx = ox + (ccx + 0.5) * res, cwy = oy + (ccy + 0.5) * res;
          double cd = std::hypot(cwx - rx, cwy - ry);
          if (cd >= min_goal_distance_ && cd < best_d && hasClearance(ccx, ccy)) {
            best_d = cd; best_wx = cwx; best_wy = cwy;
          }
          for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
              int nx = ccx + dx, ny = ccy + dy;
              if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
              int ni = idx(nx, ny);
              if (isFrontier[ni] && !visited[ni]) { visited[ni] = 1; q.push_back(ni); }
            }
        }
        if (n < min_frontier_size_) continue;
        if (best_d > 1e17) continue;               // no cell in this cluster far enough to be a goal
        if (blacklisted(best_wx, best_wy)) continue;
        double score = gain_scale_ * n - potential_scale_ * best_d;   // info-gain: bigger + closer = better
        out.push_back({best_wx, best_wy, n, best_d, score});
      }
    int nFree = 0, nUnk = 0, nFC = 0;
    for (int k = 0; k < W * H; ++k) {
      if (isFree(g.data[k])) nFree++;
      if (g.data[k] == -1) nUnk++;
      if (isFrontier[k]) nFC++;
    }
    last_frontier_cells_ = nFC; cur_free_cells_ = nFree;
    RCLCPP_INFO(get_logger(), "findFrontiers: free=%d unknown=%d frontierCells=%d clusters=%zu",
                nFree, nUnk, nFC, out.size());
    return out;
  }

  // A world point is a valid goal cell if it is FREE and has goal_clearance from any wall (so Nav2 can fit
  // the footprint there). Used to snap a clamped intermediate onto navigable ground.
  bool cellNavigable(double wx, double wy) {
    if (!map_) return false;
    const auto &g = *map_;
    const int W = g.info.width, H = g.info.height;
    const double res = g.info.resolution, ox = g.info.origin.position.x, oy = g.info.origin.position.y;
    int cx = (int)std::floor((wx - ox) / res), cy = (int)std::floor((wy - oy) / res);
    if (cx < 0 || cy < 0 || cx >= W || cy >= H) return false;
    int v = g.data[cy * W + cx];
    if (!(v >= 0 && v <= free_thresh_)) return false;
    const int clr = std::max(1, (int)std::lround(goal_clearance_ / res));
    for (int dy = -clr; dy <= clr; ++dy)
      for (int dx = -clr; dx <= clr; ++dx) {
        int nx = cx + dx, ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        if (g.data[ny * W + nx] >= occupied_thresh_) return false;
      }
    return true;
  }

  // SHORT-GOAL STEPPING: if the chosen frontier is beyond max_goal_distance, replace the goal with the
  // FARTHEST navigable, non-blacklisted cell along the robot->frontier ray within the cap -> a short,
  // reachable step. The next tick re-finds frontiers and steps again. If no step is navigable, keep the
  // frontier (Nav2 tries it; a real failure blacklists it) so we never freeze.
  void clampGoal(double &wx, double &wy, double rx, double ry) {
    double d = std::hypot(wx - rx, wy - ry);
    if (d <= max_goal_distance_ || !map_) return;
    const double res = map_->info.resolution;
    if (res <= 0.0 || max_goal_distance_ < min_goal_distance_) return;   // degenerate-config guard (no inf loop)
    const double ux = (wx - rx) / d, uy = (wy - ry) / d;
    for (double s = max_goal_distance_; s >= min_goal_distance_; s -= res) {
      double tx = rx + ux * s, ty = ry + uy * s;
      if (cellNavigable(tx, ty) && !blacklisted(tx, ty)) { wx = tx; wy = ty; return; }
    }
  }

  void sendGoal(double wx, double wy, double rx, double ry) {
    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for Nav2 action server");
      return;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = wx; goal.pose.pose.position.y = wy;
    double yaw = std::atan2(wy - ry, wx - rx);
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    cur_goal_x_ = wx; cur_goal_y_ = wy; goal_sent_time_ = now(); state_ = NAVIGATING;
    near_goal_ = false; last_x_ = rx; last_y_ = ry; last_progress_time_ = now();  // reset reactive trackers
    RCLCPP_INFO(get_logger(), "-> frontier (%.2f, %.2f)  dist=%.2f", wx, wy, std::hypot(wx - rx, wy - ry));

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
    opt.goal_response_callback = [this](GoalHandle::SharedPtr h) {
      if (!h) { RCLCPP_WARN(get_logger(), "goal rejected"); blacklist(cur_goal_x_, cur_goal_y_); state_ = EXPLORING; }
      else goal_handle_ = h;
    };
    opt.result_callback = [this](const GoalHandle::WrappedResult &r) {
      // Blacklist EVERY attempted goal -- on failure (don't retry the unreachable; Nav2 has already
      // run its collision-aware recovery before aborting) and on SUCCESS (Nav2 reports SUCCEEDED
      // within goal tolerance, so otherwise we'd re-pick it forever). Blacklist has a TTL.
      if (r.code != rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_WARN(get_logger(), "goal failed/aborted (after Nav2 recovery) -- blacklisting");
      blacklist(cur_goal_x_, cur_goal_y_);
      goal_handle_.reset();
      if (state_ == NAVIGATING) state_ = EXPLORING;  // find next frontier
    };
    nav_client_->async_send_goal(goal, opt);
  }

  void cancelGoal() { if (goal_handle_) { nav_client_->async_cancel_goal(goal_handle_); goal_handle_.reset(); } }

  // Blacklist with a TTL: an entry blocks frontiers within blacklist_radius_ for blacklist_ttl_ s,
  // then expires (so a transient failure doesn't permanently abandon a region).
  void blacklist(double x, double y) { blacklist_.push_back({x, y, now()}); }
  bool blacklisted(double x, double y) {
    for (auto &b : blacklist_)
      if ((now() - b.t).seconds() < blacklist_ttl_ && std::hypot(x - b.x, y - b.y) < blacklist_radius_) return true;
    return false;
  }

  void publishMarkers(const std::vector<Frontier> &fs) {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker del; del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);
    int id = 0;
    for (auto &f : fs) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map"; m.header.stamp = now(); m.ns = "frontiers"; m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE; m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = f.wx; m.pose.position.y = f.wy; m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.12;   // small dots
      m.color.g = 1.0; m.color.a = 0.8;
      arr.markers.push_back(m);
    }
    markers_pub_->publish(arr);
  }

  void advance(const char *why) {
    RCLCPP_INFO(get_logger(), "%s -- blacklisting (%.2f,%.2f) + replanning to a different frontier",
                why, cur_goal_x_, cur_goal_y_);
    cancelGoal(); blacklist(cur_goal_x_, cur_goal_y_); near_goal_ = false; state_ = EXPLORING;
  }

  // Reactive watchdog (each tick while NAVIGATING) -- anti-circling / anti-stuck core.
  void watchdog() {
    if (state_ != NAVIGATING) return;
    double rx, ry;
    if (robotPose(rx, ry)) {
      // ARRIVAL: near the frontier -- exploration needs no pinpoint reaching; lingering near it
      // without Nav2 completing == circling. Advance once close for arrival_patience.
      double d = std::hypot(cur_goal_x_ - rx, cur_goal_y_ - ry);
      if (d < arrival_radius_) {
        if (!near_goal_) { near_goal_ = true; near_since_ = now(); }
        else if ((now() - near_since_).seconds() > arrival_patience_) { advance("reached frontier (near, advancing)"); return; }
      } else near_goal_ = false;
      // PROGRESS: wedged for progress_timeout (Nav2's own collision-aware recovery has had its
      // chance) -> give up on this goal and try a different frontier. We do NOT drive in reverse.
      if (std::hypot(rx - last_x_, ry - last_y_) > progress_dist_) { last_x_ = rx; last_y_ = ry; last_progress_time_ = now(); }
      else if ((now() - last_progress_time_).seconds() > progress_timeout_) { advance("no progress (stuck)"); return; }
    }
    if ((now() - goal_sent_time_).seconds() > goal_timeout_s_) advance("goal timed out");  // hard cap
  }

  // params
  int min_frontier_size_, free_thresh_, occupied_thresh_;
  double min_goal_distance_, blacklist_radius_, goal_timeout_s_, planning_period_, goal_clearance_;
  double arrival_radius_, arrival_patience_, progress_dist_, progress_timeout_;
  double gain_scale_, potential_scale_, blacklist_ttl_, max_goal_distance_;
  int done_confirm_, max_clear_retries_;
  std::string robot_frame_;
  // state
  State state_{IDLE};
  int last_frontier_cells_{0}, empty_cycles_{0}, clears_done_{0}, cur_free_cells_{0}, free_high_water_{0};
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  struct BL { double x, y; rclcpp::Time t; };
  std::vector<BL> blacklist_;
  double cur_goal_x_{0}, cur_goal_y_{0};
  rclcpp::Time goal_sent_time_;
  // reactive goal tracking
  bool near_goal_{false};
  rclcpp::Time near_since_, last_progress_time_;
  double last_x_{0}, last_y_{0};
  GoalHandle::SharedPtr goal_handle_;
  // ros
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
