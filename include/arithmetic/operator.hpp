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

#ifndef ARITHMETIC__OPERATOR_HPP_
#define ARITHMETIC__OPERATOR_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"


/*
  Operator 클래스
  rclcpp::Node 클래스 상속 받는 자식 클래스
*/
class Operator : public rclcpp::Node
{
public:
  using ArithmeticOperator = msg_srv_action_interface_example::srv::ArithmeticOperator;

  // 생성자에서 rclcpp::NodeOptions를 매개변수로 가짐
  explicit Operator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Operator();

  // 서비스 요청 함수
  void send_request();

private:
  // 스마트 포인터 타입의 멤버 변수
  rclcpp::Client<ArithmeticOperator>::SharedPtr arithmetic_service_client_;
};
#endif  // ARITHMETIC__OPERATOR_HPP_
