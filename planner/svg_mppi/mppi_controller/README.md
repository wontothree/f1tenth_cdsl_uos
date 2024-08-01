# MPPI Controller Package

# ROS1 Variables and Functions

## mppi_controller_ros.cpp

|Index|Return Type|Function|Input|Description|
|---|---|---|---|---|
|1||MPPIControllerROS::MPPIControllerROS||ROS에서 MPPI 컨트롤러의 노드를 초기화하는 생성자 함수다. 이 함수는 노드가 시작될 때 여러 파라미터를 로드하고 다양한 ROS 컴포넌트를 설정하며 필요한 데이터 스트림을 구독하고 게시할 준비를 한다.|
|2|void|MPPIControllerROS::callback_odom_with_pose|const nav_msgs::Odometry& odom|Odometry 정보(선형 속도, 각속도)를 기반으로 로봇의 위치와 자세를 추정한다.|
|3|void|MPPIControllerROS::callback_odom|const nav_msgs::Odometry& odom|Odom 정보를 입력받아 로봇의 현재 속도 정보를 업데이트하는 함수. 위치 추정 없이 로봇의 속도만을 사용하는 간단한 형태의 함수. 주로 로봇의 속도 정보만 필요로 하는 시스템에서 사용된다.|
|4|void|MPPIControllerROS::callback_activate_signal|const std_msgs::Bool& is_activate|ROS에서 std_msgs::Bool 타입의 메시지를 입력받아 내부 상태 플래그를 업데이트하는 콜백 함수. 이 함수는 로봇 시스템의 활성화 상태를 제어하는 . 데사용된다.|
|5|void|MPPIControllerROS::callback_grid_map|const grid_map_msgs::GridMap& grid_map|ROS 환경에서 grid_map_msgs::GridMap 타입의 메시지를 입력받아 내부 장애물 지도를 업데이트하는 콜백 함수이다. 이 함수는 로봇의 경로 계획 및 충돌 회피와 관련된 작업에 사용된다.|
|6|void|MPPIControllerROS::callback_reference_sdf|const grid_map_msgs::GridMap& grid_map|그리드 맵 형태로 참조 경로 정보가 포함된 grid_map을 입력받아 참조 경로에 대한 서명 거리 함수 데이터를 업데이트하는 콜백 함수이다.|
|7|void|MPPIControllerROS::start_cmd_callback|[[maybe_unused]] const std_msgs::Empty& msg|로봇이 특정 작업을 시작하도록 명령을 전달받았을 때 사용된다. 로봇의 특정 동작 모드를 활성화하거나 경로 계획 알고리즘을 시작하는 등의 작업을 수행할 수 있다.|
|8|void|MPPIControllerROS::stop_cmd_callback|[[maybe_unused]] const std_msgs::Empty& msg|std_msgs::Empty 타입의 메시지를 입력받아 내부 상태를 업데이트하는 콜백 함수. 이. ㅏㅁ수는 로봇의 동작을 중지하도록 하는 신호를 수신했을 때 호출된다.|
|9|void|MPPIControllerROS::timer_callback|[[maybe_unused]] const ros::TimerEvent& te|ROS의 타이머 콜백 함수로 주기적으로 호출되어 로봇의 제어 및 경로 계획을 처리한다. 이 함수는 로봇의 상태를 확인하고 제어 명령을 계산하여 발행하며 디버깅 및 시각화 정보를 제공한다.|
|10|void|MPPIControllerROS::publish_traj|const mppi::cpu::StateSeq& state_seq, const std::string& name_space, const std::string& rgb, const ros::Publisher& publisher|로봇의 경로 또는 제어 명령의 예측 결과를 시각화하기 위해 visualization_msgs::MarkerArray를 사용하여 마커를 발행하는 함수이다. 이 함수는 ROS의 RViz에서 경로를 시각적으로 표시하는 데 유용하다.|
|11|void|MPPIControllerROS::publish_path|const mppi::cpu::StateSeq& state_seq, const std::string& name_space, const std::string& rgb, const ros::Publisher& publisher|로봇의 경로를 시각화하기 위해 RViz에서 사용할 수 있는 마커를 발행하는 역할을 한다. 이 함수는 mppi:cpu:StateSeq 타입의 상태 시퀀스를 입력으로 받아 경로를 선과 구 형태로 시각화하여 RViz에 발행한다.|
|12|void|MPPIControllerROS::publish_candidate_paths|const std::vectormppi::cpu::StateSeq& state_seq_batch, const std::vector<double>& weights, const ros::Publisher& publisher|여러 경로 후보들을 시각화하기 위해 RViz에서 사용할 마커들을 발행하는 역할을 한다. 이 함수는 MPPI 컨트롤러가 계산한 여러 후보 경로를 시각적으로 표현하여 로봇이 선택할 수 있는 경로들을 분석하는 데 도움을 준다.|
|13|void|MPPIControllerROS::publish_control_covs|const mppi::cpu::StateSeq& state_seq, const mppi::cpu::ControlSeqCovMatrices& cov_matrices, const ros::Publisher& publisher|MPPI 컨트롤러의 제어 입력 공분산을 RViz에서 시각화하기 위한 마커를 발행하는 역할을 한다. 이 함수는 주로 제어 입력의 불확실성을 시각적으로 표현하여 디버깅 및 분석에 도움을 준다.|
|14|void|MPPIControllerROS::publish_state_seq_dists|const mppi::cpu::StateSeq& state_seq, const mppi::cpu::XYCovMatrices& cov_matrices, const ros::Publisher& publisher|MPPI 컨트롤러의 상태 시퀀스와 관련된 분포를 시각화하기 위해 사용된다. 이. ㅏㅁ수는 상태 시퀀스의 각 상태에 대해 공분산을 기반으로 한 타원 마커를 RViz에서 발행하여 상태의 불확실성을 시각적으로 표현한다.|

## mpc_base.cpp

|Index|Return Type|Function|Input|Description|
|---|---|---|---|---|
|1||MPCBase|const Params::Common& params, const size_t& sample_num|MPCBase 클래스의 생성자 함수. MPC 알고리즘의 기본 설정 및 초기화를 수행한다.|
|2|void|MPCBase::set_obstacle_map|const grid_map::GridMap& obstacle_map|장애물 맵의 설정을 간단하게 수행할 수 있게 한다. 장애물 회피와 관련된 로직에서 장애물 정보를 일관되게 사용할 수 있도록 보장한다.|
|3|void|MPCBase::set_reference_map|const grid_map::GridMap& reference_map|참조 맵을 설정하는 함수. 차량의 경로 계획과 목표 위치 설정에 중요한 정보를 제공한다. 클래스 내에서 참조 맵을 활용하여 최적의 경로를 계획하고 목표를 달성하는 데 기여한다.|
|4|std::pair<std::vector<double>, std::vector<double>>|MPCBase::calc_sample_costs|const PriorSamplesWithCosts& sampler, const State& init_state|샘플들의 비용을 계산하는 함수. 현재 모드에 따라 로컬라이즈 모드 또는 일반 모드에서 호출된다. 로컬라이즈 모드에서는 로컬 상태 시퀀스와 장매물 맵을 기반으로 비용을 계산하고 일반 모드에서는 글로벌 상태 시퀀스와 참조 맵을 포함하여 비용을 계산한다. 이 함수는 최적의 경로를 찾기 위해 다양한 샘플의 비용을 평가하는 데 사용된다.|
|5|std::tuple<StateSeq, double, double>|MPCBase::get_predictive_seq|const State& initial_state, const ControlSeq& control_input_seq|주어진 초기 상태와 제어 입력 시퀀스를 사용하여 예측 상태 시퀀스를 계산하고 이와 관련된 비용을 반환하는 역할을 한다. 이 함수는 두 가지 모드(로컬라이즈 모드와 일반 모드)에 따라 다르게 동작한다.|
|6|std::pair<std::vector<StateSeq>, std::vector<double>>|MPCBase::get_state_seq_candidates|const int& _num_samples, const std::vector<double>& weights|주어진 샘플 수와 가중치를 바탕으로 가장 유망한 상태 시퀀스 후보를 선택하여 반환한다. 이 함수는 가중치를 기준으로 가장 높은 값을 가진 상태 시퀀스를 선택하여 반환한다.|
|7|std::pair<StateSeq, XYCovMatrices>|MPCBase::get_proposed_distribution|void|현재 상태 시퀀스 후보 집합에서 제안된 상태 분포와 공분산 행렬을 계산하여 반환하는 함수|
|8|std::pair<std::vector<double>, std::vector<double>>|MPCBase::calc_sample_costs|const PriorSamplesWithCosts& sampler, const State& local_init_state, const grid_map::GridMap& obstacle_map, StateSeqBatch* local_state_seq_candidates|여러 파라미터를 통해 함수가 상태 시퀀스를 예측하고 비용을 계산ㅎ며 특정 상황에서 상태 시퀀스를 평가하는 데 필요한 정보를 제공한다. 이 함수는 MPC 문제에서 상태 시퀀스를 평가하고 최적화하기 위해 필수적인 역할을 한다.|
|9|std::pair<std::vector<double>, std::vector<double>>|MPCBase::calc_sample_costs|const PriorSamplesWithCosts& sampler, const State& local_init_state, const grid_map::GridMap& obstacle_map, StateSeqBatch* local_state_seq_candidates|주어진 제어 시퀀스 샘플들에 대해 비용을 계산하는 기능을 수행한다. 이 함수는 제어 시퀀스의 예측된 상태 시퀀스를 바탕으로 각각의 제어 시퀀스에 대한 총 비용과 충돌 비용을 계산한다.|
|10|StateSeq|MPCBase::predict_state_seq|const ControlSeq& control_seq, const State& init_state, const grid_map::GridMap& reference_map|주어진 제어 시퀀스에 따라 차량의 상태 시퀀스를 예측하는 기능을 수행한다. 이 함수는 차량의 동작을 예측하기 위해 주어진 제어 입력(조향각 등)과 초기 상태를 바탕으로 상태 시퀀스를 계산한다.|
|11|void|MPCBase::predict_state_seq|const ControlSeq& control_seq, const State& global_init_state, const grid_map::GridMap& reference_map, StateSeq* global_state_seq, StateSeq* local_state_seq|제어 시퀀스와 초기 상태를 사용하여 차량의 전역 및 지역 상태 시퀀스를 예측하는 함수. 예측할 상태 시퀀스를 두 가지로 나누어 계산한다. 전역 상태 시퀀스와 지역 상태 시퀀스. 이 함수는 차량의 동적 모델을 기반으로 상태를 업데이트하며, 차량의 제어 입력과 상태 변화에 대한 예측을 수행한다.|
|12|double|MPCBase::constant_speed_prediction|const double& current_speed|주어진 속도를 그대로 반환하는 함수. 차량의 속도를 예측할 때 속도가 일정하게 유지된다고 가정할 때 사용된다. 즉 속도가 현재 속도와 동일하게 유지된다고 예측한다.|
|13|double|MPCBase::linear_speed_prediction|const double& current_speed, const double& target_speed, const double& prediction_interval, const double& min_accel, const double& max_accel|차량의 속도를 예측할 때 현재 속도에서 목표 속도까지 선형적으로 변화한다고 가정한다. 이 함수는 차량이 목표 속도에 도달하기 위해 최소 또는 최대 가속도를 사용하여 속도를 조정하는 방식으로 작동한다.|
|14|double|MPCBase::reference_speed_prediction|const double& pos_x, const double& pos_y, const grid_map::GridMap& reference_map|주어진 위치에서 참조 맵을 사용하여 속도를 예측하는 함수. 위치가 참조 맵의 내부에 있는지 확인하고, 그. ㅟ치가 정의된 속도를 반환한다.|
|15|std::pair<double, double>|MPCBase::state_cost|const StateSeq& local_state_seq, const grid_map::GridMap& obstacle_map|local_state_seq로 제공되는 예측된 상태 시퀀스와 장애물 지도를 기반으로 충돌과 관련된 총 비용을 계산한다. 이 비용은 모델 예측 제어에서 다양한 상태 시퀀스의 품질을 평가하는 . 데사용되며, 알고리즘이 충돌 위험을 최소화하는 궤적을 선택할 수 있게 한다.|
|16|std::pair<double, double>|MPCBase::state_cost|const StateSeq& global_state_seq, const StateSeq& local_state_seq, const grid_map::GridMap& local_obstacle_map, const grid_map::GridMap& ref_path_map|예측된 경로와 참조 경로를 비교하여 비용을 계산ㅎ고 장애물과의 충돌 가능성을 평가하여 경로의 안전성을 판단한다. 이 함수는 경로가 얼마나 잘 따르고 있는지를 나타내는 참조 비용과 장애물 충돌 위협을 평가하는 충돌 비용을 계산하여 반환한다.|
|17|std::pair<StateSeq, XYCovMatrices>|MPCBase::calc_state_distribution|const StateSeqBatch& state_seq_candidates|여러 상태 시퀀스 후보들로부터 평균 상태 시퀀스를 계산하고, 각 예측 단계에서 x와 y 위치의 공분산 행렬을 계산하는 함수이다. 이 함수는 MPC에서 경로 예측의 불확실성을 평가하고, 상태 분포를 분석하기 위해 사요된다.|
