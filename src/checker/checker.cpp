// Copyright 2021 OROCA
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

#include <memory>

#include "checker/checker.hpp"

// Checker 노드 생성자
Checker::Checker(float goal_sum, const rclcpp::NodeOptions & node_options)
: Node("checker", node_options) // 부모 클래스인 Node를 노드명(checker), node_options 인자로 초기화
{
  // create_client 함수로 노드의 인터페이스들과 액션명을 인자로 받아 rclcpp_action::Client 실체화
  arithmetic_action_client_ = rclcpp_action::create_client<ArithmeticChecker>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "arithmetic_checker");

  send_goal_total_sum(goal_sum);  // 액션 목표값 전송 함수
}

Checker::~Checker()
{
}

// goal_sum 변수를 인자로 받아 액션 목표값을 전송
void Checker::send_goal_total_sum(float goal_sum)
{
  using namespace std::placeholders;

  // 액션 클라이언트 실체화 확인
  if (!this->arithmetic_action_client_) {
    RCLCPP_WARN(this->get_logger(), "Action client not initialized");
  }
  // 액션 서버와의 통신 확인
  if (!this->arithmetic_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_WARN(this->get_logger(), "Arithmetic action server is not available.");
    return;
  }

  // 액션 목표값 설정 
  // ArithmeticChecker 액션 인터페이스의 Goal 메시지를 불러와 인자로 받은 goal_sum 변수를 넘겨줌
  auto goal_msg = ArithmeticChecker::Goal();
  goal_msg.goal_sum = goal_sum;


  // rclcpp_action::Client의 SendGoalOptions 구조체를 통해 각 함수 초기화
  auto send_goal_options = rclcpp_action::Client<ArithmeticChecker>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&Checker::get_arithmetic_action_goal, this, _1);
  
  send_goal_options.feedback_callback =
    std::bind(&Checker::get_arithmetic_action_feedback, this, _1, _2);
  
  send_goal_options.result_callback =
    std::bind(&Checker::get_arithmetic_action_result, this, _1);
  
  // 액션 서버에 Goal 메시지와 함께 전송
  this->arithmetic_action_client_->async_send_goal(goal_msg, send_goal_options);
}


// SendGoalOptions의 goal_response_callback
// 액션 서버의 handle_goal 함수와 연결
// 액션 목표값 전송 후 첫번째 응답 확인
void Checker::get_arithmetic_action_goal(
  std::shared_future<GoalHandleArithmeticChecker::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Action goal rejected.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Action goal accepted.");
  }
}

// SendGoalOptions의 feedback_callback
// 액션 서버의 execute_checker 함수와 연결
// 액션 인터페이스의 feedback 메시지를 인자로 가지고 있어 액션 피드백 확인 가능
void Checker::get_arithmetic_action_feedback(
  GoalHandleArithmeticChecker::SharedPtr,
  const std::shared_ptr<const ArithmeticChecker::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Action feedback: ");
  for (const auto & formula : feedback->formula) {
    RCLCPP_INFO(this->get_logger(), "\t%s ", formula.c_str());
  }
}

// SnedGoalOptions의 result_callback
// 액션 서버의 execute_checker 함수와의 연결 
void Checker::get_arithmetic_action_result(
  const GoalHandleArithmeticChecker::WrappedResult & result) // 액션 결괏값 확인, 액션 결과 상태에 따른 액션 클라리언트의 대응 로직 개발 가능ㄴ
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action succeeded!");
      RCLCPP_INFO(this->get_logger(), "Action result(all formula): ");
      for (const auto & formula : result.result->all_formula) {
        RCLCPP_INFO(this->get_logger(), "\t%s ", formula.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "Action result(total sum): ");
      RCLCPP_INFO(this->get_logger(), "\t%.2f ", result.result->total_sum);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "The action was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "The action was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}
