# local_planner

    local_planner
    ├── include/                                    # hpp files
    |     └── local_planner                           
    |         ├── local_planner_ros,hpp             # ros program
    |         ├── mpc_types.hpp                     # data types definition for mpc
    |         ├── mpc_template.hpp                  # abstract class (interface) for MPC implementation
    |         └── mpc_base.hpp                      # essential building blocks for implementing MPC
    <!-- |         └── mpc_solver_svg_mppi.hpp           # svg mppi solver -->
    ├── src/                                        # cpp files
    |     ├── local_planner_node.cpp                # running local_planner_node
    |     ├── local_planner_ros.cpp
    |     ├── mpc_base.cpp                 
    <!-- |     └── mpc_solver_svg_mppi.cpp                 -->
    ├── CMakeLists.txt                       
    ├── package.xml                           

# Dependencies

```bash
sudo apt-get install libeigen3-dev
```

# Functions

- local_planner_ros.hpp

|Return|Function Name|Input|Description|Dependencies|
|---|---|---|---|---|
||callback_local_costmap||||
||callback_odometry||||
||callback_timer||||

- mpc_template.hpp : MPC 알고리즘을 구현을 위한 여러 가지 기능을 정의하는 추상 클래스 파일

|Return|Function Name|Input|Description|Dependencies function|
|---|---|---|---|---|
||~MPCTemplate()||Deconstructor||
|std::pair<ControlSeq, double>|solve|const State& initial_state|시스템의 초기 상태를 입력으로 받아 업데이트된 제어 시퀀스와 충돌률을 반환하는 solver 함수||
|void|set_local_costmap|const grid_map::GridMap& local_costmap|local costmap 설정 함수||
|void|set_global_costmap|const grid_map::GridMap& global_costmap|global costmap 설정 함수||
|ControlSeq|get_control_seq|void|현재 사용 중인 제어 시퀀스를 반환한다.||
|std::pair<std::vector<StateSeq>, std::vector<double>>|get_state_seq|const int& num_samples|주어진 제어 입력 시퀀스를 기반으로 상태 시퀀스를 예측하고, 이에 대한 비용과 입력 오차를 계산한다.||
||||||
|std::tuple<StateSeq, double, double, double>|get_predictive_seq|const State& initial_state, const ControlSeq& control_input_seq|주어진 제어 입력 시퀀스를 기반으로 상태 시퀀스를 예측하고 이에 대한 비용과 입력 오차를 계산한다.||
|ControlSeqCovMatrices|get_cov_matrices|void|현재 샘플의 공분산 행렬을 반환한다.||
|std::pair<StateSeq, XYCovMatrices>|get_proposed_state_distribution|void|제안된 상태 분포를 반환한다.||
|std::vector<double>|softmax||||

- mpc_base.hpp

|Return|Function Name|Input|Description|Dependencies function|
|---|---|---|---|---|
||set_local_costmap|const grid_map::GridMap& local_costmap|||
||set_global_costmap|const grid_map::GridMap& global_costmap|||
||calc_sample_costs||||
||get_predictive_seq||||
||get_state_seq_candidates||||
||get_proposed_distribution||||
||calc_sample_costs||||
||calc_sample_costs||||
||predict_state_seq||||
||predict_state_seq||||
||constant_speed_prediction||||
||linear_speed_prediction||||
||linear_speed_prediction||||
||reference_speed_prediction||||
||state_cost||||
||state_cost||||
||calc_state_distribution||||

# Variables

|Type|Variable Name|Description|Initialization|Dependencies|
|---|---|---|---|---|
|bool|is_localize_less_mode_|localize 모드 여부||
|std::string|topic_name_local_costmap_|||string|
|std::string|topic_name_odometry_|||string|
|rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr|sub_local_costmap_|local costmap에 대한 topic subscriber 객체 smart pointor||grid_map_msgs/msg/grid_map.hpp|
|rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr|sub_odometry_|lodometry에 대한 topic subscriber 객체 smart pointer||nav_msgs/msg/odometry.hpp|
|bool|is_local_costmap_received_||false||
|bool|is_odometry_received_||false||
||||||
|rclcpp::TimerBase::SharedPtr|timer_|타이머 참조 변수||rclcpp|
|std::string|mpc_mode_|||string|
||mpc_solver_|||
