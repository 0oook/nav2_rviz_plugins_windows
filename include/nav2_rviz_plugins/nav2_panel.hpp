// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "std_srvs/srv/trigger.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_rviz_plugins/ros_action_qevent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{

class InitialThread;

/// Panel to interface to the nav2 stack
class Nav2Panel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Nav2Panel(QWidget * parent = 0);
  virtual ~Nav2Panel();

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void logPressedButton();

private Q_SLOTS:
  void startThread();
  void manageLifecycleManagedNodes(const unsigned char &command);
  void onStartup();
  void onShutdown();
  void onCancel();
  void onPause();
  void onResume();
  void onAccumulatedWp();
  void onAccumulatedNTP();
  void onAccumulating();
  void onNewGoal(double x, double y, double theta, QString frame);

private:
  void loadLogFiles();
  void onCancelButtonPressed();
  void timerEvent(QTimerEvent * event) override;

  int unique_id {0};

  // Call to send NavigateToPose action request for goal poses
  void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses);
  void startNavigation(geometry_msgs::msg::PoseStamped);
  void startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses);
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  using NavThroughPosesGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;

  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
  rclcpp::CallbackGroup::SharedPtr cg_{nullptr};
  std::unique_ptr<std::thread> thread_;

  // Timeout value when waiting for action servers to respnd
  std::chrono::milliseconds server_timeout_;

  // A timer used to check on the completion status of the action
  QBasicTimer timer_;

  // The NavigateToPose action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    nav_through_poses_action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lifecycle_manager_navigation_is_active_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lifecycle_manager_localization_is_active_client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_manager_navigation_manage_nodes_client_;

  // Navigation action feedback subscribers
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    navigation_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
    nav_through_poses_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
    navigation_goal_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr
    nav_through_poses_goal_status_sub_;

  // Goal-related state
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;

  QPushButton * start_reset_button_{nullptr};
  QPushButton * pause_resume_button_{nullptr};
  QPushButton * navigation_mode_button_{nullptr};

  QLabel * navigation_status_indicator_{nullptr};
  QLabel * localization_status_indicator_{nullptr};
  QLabel * navigation_goal_status_indicator_{nullptr};
  QLabel * navigation_feedback_indicator_{nullptr};

  QStateMachine state_machine_;
  InitialThread * initial_thread_;

  QState * pre_initial_{nullptr};
  QState * initial_{nullptr};
  QState * idle_{nullptr};
  QState * reset_{nullptr};
  QState * paused_{nullptr};
  QState * resumed_{nullptr};
  // The following states are added to allow for the state of the button to only expose reset
  // while the NavigateToPoses action is not active. While running, the user will be allowed to
  // cancel the action. The ROSActionTransition allows for the state of the action to be detected
  // and the button state to change automatically.
  QState * running_{nullptr};
  QState * canceled_{nullptr};
  // The following states are added to allow to collect several poses to perform a waypoint-mode
  // navigation or navigate through poses mode.
  QState * accumulating_{nullptr};
  QState * accumulated_wp_{nullptr};
  QState * accumulated_nav_through_poses_{nullptr};

  std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

  // Publish the visual markers with the waypoints
  void updateWpNavigationMarkers();

  // Create unique id numbers for markers
  int getUniqueId();

  void resetUniqueId();

  // create label string from goal status msg
  static inline QString getGoalStatusLabel(
    int8_t status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN);

  // create label string from feedback msg
  static inline QString getNavToPoseFeedbackLabel(
    nav2_msgs::action::NavigateToPose::Feedback msg =
    nav2_msgs::action::NavigateToPose::Feedback());
  static inline QString getNavThroughPosesFeedbackLabel(
    nav2_msgs::action::NavigateThroughPoses::Feedback =
    nav2_msgs::action::NavigateThroughPoses::Feedback());
  template<typename T>
  static inline std::string toLabel(T & msg);

  // round off double to the specified precision and convert to string
  static inline std::string toString(double val, int precision = 0);

  // Waypoint navigation visual markers publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
};

class InitialThread : public QThread
{
  Q_OBJECT

public:
  explicit InitialThread(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_loc,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_nav
  )
  : client_loc_(std::move(client_loc)), client_nav_(std::move(client_nav))
  {
    request_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    rater_ = std::make_shared<rclcpp::WallRate>(10);
  }

  void run() override
  {

    while (rclcpp::ok()) {
      rater_->sleep();
      if (!client_loc_->wait_for_service(std::chrono::seconds(1))) {
        emit localizationInactive();
      }
      else {
        auto fut = client_loc_->async_send_request(request_);
        if (fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
          if (fut.future.get()->success) {
            emit localizationActive();
          }
          else {
            emit localizationInactive();
          }
        }
        else {
          emit localizationInactive();
        }
      }

      if (!client_nav_->wait_for_service(std::chrono::seconds(1))) {
        emit navigationInactive();
      }
      else {
        auto fut = client_nav_->async_send_request(request_);
        if (fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
          if (fut.future.get()->success) {
            emit navigationActive();
          }
          else {
            emit navigationInactive();
          }
        }
        else {
          emit navigationInactive();
        }
      }
    }
  }

signals:
  void navigationActive();
  void navigationInactive();
  void localizationActive();
  void localizationInactive();

private:
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_loc_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_nav_;
  rclcpp::WallRate::SharedPtr rater_;
};

}  // namespace nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
