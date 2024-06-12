# Stein Variational Guided Model Predictive Path Integral Control

## [ROS2] Nav2 MPPI

### Dependencies

Buildtool Dependencies

- ament_cmake: ROS 2에서 CMake 기반의 빌드 시스템을 설정하고 관리하는 도구입니다. 기본적인 빌드 설정을 제공하여 패키지를 빌드할 수 있게 해줍니다.
- ament_cmake_ros: ament_cmake의 ROS 확장으로, ROS 2 패키지 빌드를 위한 추가적인 기능과 매크로를 제공합니다.

Package Dependencies

- rclcpp: ROS 2의 C++ 클라이언트 라이브러리로, ROS 2 노드를 생성하고 통신할 수 있게 해줍니다.
- *nav2_common*: 네비게이션 2 (Navigation 2) 패키지에서 공통적으로 사용되는 유틸리티와 설정을 포함합니다.
- *nav2_core*: 네비게이션 2의 핵심 인터페이스를 정의하는 패키지입니다.
- *nav2_util*: 네비게이션 2에서 자주 사용되는 유틸리티 함수와 클래스들을 포함합니다.
- *nav2_costmap_2d*: 2D 코스트맵을 관리하고 업데이트하는 네비게이션 2 패키지입니다.
- geometry_msgs: 기하학적 데이터 타입(포즈, 트위스트 등)을 정의하는 메시지 타입 패키지입니다.
- visualization_msgs: 시각화 도구를 위한 메시지 타입을 제공하는 패키지입니다.
- *nav2_msgs*: 네비게이션 2에서 사용하는 커스텀 메시지 타입을 정의합니다.
- pluginlib: ROS 2에서 플러그인 기반의 아키텍처를 지원하는 라이브러리입니다.
- tf2_geometry_msgs: tf2 라이브러리와 geometry_msgs 타입 간의 변환을 지원하는 패키지입니다.
- tf2: 좌표 변환을 위한 라이브러리로, 여러 프레임 간의 변환을 관리합니다.
- tf2_eigen: tf2와 Eigen 라이브러리 간의 변환을 지원하는 패키지입니다.
- tf2_ros: tf2 라이브러리의 ROS 인터페이스를 제공하는 패키지입니다.
- std_msgs: 표준 메시지 타입을 정의하는 패키지로, 기본적인 데이터 타입을 제공합니다 (예: Int32, String 등).
- xtensor: C++에서 사용되는 텐서(다차원 배열) 라이브러리입니다.
- libomp-dev: OpenMP (Open Multi-Processing)를 위한 라이브러리로, 멀티스레딩을 지원합니다.
- benchmark: 성능 벤치마킹을 위한 라이브러리입니다.
- xsimd: SIMD (Single Instruction, Multiple Data) 명령어를 활용한 벡터화 라이브러리입니다.

Test Dependencies

- ament_lint_auto: 다양한 린트 도구들을 자동으로 실행하여 코드 품질을 검사하는 패키지입니다.
- ament_lint_common: 일반적인 린트 규칙과 설정을 제공하는 패키지입니다.
- ament_cmake_gtest: Google Test 프레임워크를 사용하여 C++ 테스트를 작성하고 실행할 수 있게 해주는 도구입니다.

### Files : /src

|- critics
    |- /constraint_critic.cpp : 제약 조건 평가하는 비판자
    |- /cost_critic.cpp : 비용을 병가하는 비판자
    |- /goal_angle_critic.cpp : 목표 각도를 평가하는 비판자
    |- /goal_critic.cpp : 목표를 평가하는 비판자
    |- /obstacles_critic.cpp : 장애물을 평가하는 비판자
    |- /path_align_critic.cpp
    |- /path_align_legacy_critic.cpp
    |- /path_angle_critic.cpp
    |- /path_follow_critic.cpp
    |- /prefer_forward_critic.cpp
    |- /twirling_critic.cpp
    |- /velocity_deadband_critic.cpp
|- controller.cpp : 주어진 상황에서 제어 입력을 생성하는 주요 컨트롤러를 구현한다.
|- /critic_manager.cpp : 비판자들을 관리하고 조정하는 클래스
|- /noise_generator.cpp : 경로 생성 알고리즘에서 확률적인 요소를 추가하는 데 사용되는 노이즈를 생성한다.
|- /optimizer.cpp : 경로 최적화 알고리즘을 구현한다.
|- /parameters_handler.cpp : 시스템의 매개변수를 관리한다.
|- /path_handler.cpp : 경로를 관리하고 조직화하는 클래스와 함수를 포함한다.
|- /trajectory_visualizer.cpp : 경로를 시각화한다.

## [ROS1] SVG-MPPI
