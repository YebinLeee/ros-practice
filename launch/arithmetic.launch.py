#!/usr/bin/env python3
# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LaunchConfiguration 클래스 이용하여 실행 관련 설정을 선언
    param_dir = LaunchConfiguration(   
        'param_dir',
        default=os.path.join(
            # topic_~ 패키지의 param 폴더에 위치한 yaml 파라미터 설정파일
            get_package_share_directory('topic_service_action_rclcpp_example'),
            'param',
            'arithmetic_config.yaml'))

    # 메소드의 리턴값으로 LaunchDescription 반환
    return LaunchDescription([
        DeclareLaunchArgument( 
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file'),
        # 실행할 노드 설정
        Node(
            package='topic_service_action_rclcpp_example',  # 실행할 패키지 이름
            executable='argument',                          # 실행 가능한 노드의 이름
            name='argument',                                # 지정한 노드를 실행할 때 실제로 사용할 이름
            parameters=[param_dir],                         # yaml 파일 사용
            output='screen'),                               # screen: 로깅 정보가 기록되고 터미널 창에도 표시
        Node(
            package='topic_service_action_rclcpp_example',
            executable='calculator',
            name='calculator',
            parameters=[param_dir],
            output='screen'),
        # remappings 기능 - 특정 이름 변경
        Node(
            package='topic_service_action_rclpy_example',
            executable='argument',
            name='argument',
            remappings=[
                ('/arithmetic_argument', '/argument'), # /arithmetic_argument 토픽 이름을 /argument 로 변경
            ])
    ])