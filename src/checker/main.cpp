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

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h" // main 문의 인자를 쉽게 확인 가능

#include "checker/checker.hpp"


void print_help()
{
  printf("For Node node:\n");
  printf("node_name [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

int main(int argc, char * argv[])
{

  // '-h' 인자가 있는지 확인
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  // main 함수로 넘겨받은 argc, argv 인자를 다시 넘겨주어 '--ros-args' 인자를 rclcpp가 확인할 수 있도록 해줌
  rclcpp::init(argc, argv);

  float goal_total_sum = 50.0;
  
  // 실행 인자를 확인하고 그 값을 문자열 포인터로 반환
  // 사용자는 문자열 포인터를 원하는 변수 타입으로 변경하여 노드의 생성인자로 넘겨줄 수 있게 됨
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-g"); 
  if (nullptr != cli_option) {
    goal_total_sum = std::stof(cli_option);
  }
  printf("goal_total_sum : %2.f\n", goal_total_sum);

  auto checker = std::make_shared<Checker>(goal_total_sum);

  rclcpp::spin(checker);

  rclcpp::shutdown();

  return 0;
}
