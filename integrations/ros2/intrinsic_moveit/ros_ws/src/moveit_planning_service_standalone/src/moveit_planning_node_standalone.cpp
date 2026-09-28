// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Derived from intrinsic-ai/intrinsic-moveit
// moveit_planning_service/src/moveit_planning_node.cpp @ c5e3290; Flowstate World sync,
// Zenoh/pubsub, RuntimeContext proto and status monitor removed.

#include <atomic>
#include <chrono>
#include <memory>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_planning_interfaces/srv/plan_grasps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "moveit_planning_service/grasp_planning_pipeline.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

using PlanGrasps = moveit_planning_interfaces::srv::PlanGrasps;
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

namespace {

void handle_motion_plan_request(
    const std::shared_ptr<std::atomic<bool>>& scene_ready,
    const std::shared_ptr<GetMotionPlan::Request>& request,
    std::shared_ptr<GetMotionPlan::Response>& response) {
  if (!scene_ready->load()) {
    RCLCPP_WARN(rclcpp::get_logger("moveit_planning_service"),
                "Rejecting motion planning request: collision scene is not ready yet.");
    response->motion_plan_response.error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
              "Received motion planning request for group: '%s'",
              request->motion_plan_request.group_name.c_str());
  // Upstream is a TODO stub as well: it returns SUCCESS with an empty trajectory.
  response->motion_plan_response.group_name = request->motion_plan_request.group_name;
  response->motion_plan_response.planning_time = 0.05;
  response->motion_plan_response.error_code.val =
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
              "Successfully generated dummy motion plan.");
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_planning_node");

  rcl_interfaces::msg::ParameterDescriptor double_desc;
  double_desc.dynamic_typing = true;
  node->declare_parameter("ros_service_call_timeout_sec", rclcpp::ParameterValue(5.0),
                          double_desc);
  node->declare_parameter<bool>("expect_collision_objects", true);
  node->declare_parameter<bool>("use_mock_hardware", false);

  if (!node->get_parameter("use_mock_hardware").as_bool()) {
    RCLCPP_WARN(node->get_logger(),
                "This standalone build only supports use_mock_hardware:=true "
                "(no Intrinsic platform connection). Continuing in mock mode.");
  }
  auto scene_ready = std::make_shared<std::atomic<bool>>(true);

  auto planning_scene_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto planning_scene_client = node->create_client<GetPlanningScene>(
      "/get_planning_scene", rclcpp::ServicesQoS(), planning_scene_callback_group);
  auto planning_scene_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  planning_scene_executor->add_callback_group(planning_scene_callback_group,
                                              node->get_node_base_interface());
  std::thread planning_scene_executor_thread(
      [planning_scene_executor]() { planning_scene_executor->spin(); });

  auto planning_services_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto motion_service = node->create_service<GetMotionPlan>(
      "motion_planning/get_motion_plan",
      [scene_ready](const std::shared_ptr<GetMotionPlan::Request> request,
                    std::shared_ptr<GetMotionPlan::Response> response) {
        handle_motion_plan_request(scene_ready, request, response);
      },
      rclcpp::ServicesQoS(), planning_services_callback_group);

  auto grasp_pipeline = std::make_shared<moveit_planning_service::GraspPlanningPipeline>(
      node, planning_scene_client);

  auto grasp_service = node->create_service<PlanGrasps>(
      "grasp_planning/plan_grasps",
      [node, grasp_pipeline](const PlanGrasps::Request::SharedPtr request,
                             PlanGrasps::Response::SharedPtr response) {
        double timeout_sec = 5.0;
        node->get_parameter("ros_service_call_timeout_sec", timeout_sec);
        grasp_pipeline->PlanGrasps(
            request, response,
            std::chrono::milliseconds(static_cast<int64_t>(timeout_sec * 1000.0)));
      },
      rclcpp::ServicesQoS(), planning_services_callback_group);

  RCLCPP_INFO(node->get_logger(), "MoveIt Planning Service (standalone) started.");
  RCLCPP_INFO(node->get_logger(), " - motion_planning/get_motion_plan");
  RCLCPP_INFO(node->get_logger(), " - grasp_planning/plan_grasps");

  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  executor.spin();

  planning_scene_executor->cancel();
  if (planning_scene_executor_thread.joinable()) {
    planning_scene_executor_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
