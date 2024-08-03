# MPPI Controller Package

## config/mppi_controller.yaml

|Index|Category|Parameter|Value|Description|
|---|---|---|---|---|
|1|System|control_cmd_topic|drive|제어 명령을 보낼 주제(topic) 이름|
|2||in_reference_sdf_topic|reference_sdf|참조 SDF(Sample Data Format) 메시지를 받을 주제|
|3||in_odom_topic|odom|위치 정보를 받을 주제|
|4||is_activate_ad_topic|is_active_ad|자율 주행 모드 활성화 여부를 나타내는 주제|
|5||robot_frame_id|ego_racecar/base_link|로봇의 프레임 ID. 좌표 변환 시 사용된다.|
|6||map_frame_id|map|맵의 프레임 ID. 좌표 변환 시 사용된다.|
|7||costmap_id|f1_costmap_2d/f1_costmap/costmp|사용될 global costmap의 id. used only not localize_less_mode|
|8||local_costmap_id|local_costmap|사용될 로컬 코스트맵의 ID. used only localize_less_mode|
|9||backward_point_topic|backward_point|후진 지점을 나타내는 주제|
|10||control_sampling_time|0.025|제어 명령의 샘플링 시간 간격|
|11||use_local_costmap|true|로컬 코스트맵 사용 여부를 나타낸다. true일 경우 더 정확한 로컬 코스트맵을 사용한다. true : grid map의 local costmap을 사용한다. false : costmap 2d의 global costmap을 사용한다. local costmap이 global costmap보다 더 정확하기 때문에 local costmap을 사용할 것을 권장한다. 만약 localize_less_mode를 true로 한다면, use_local_costmap이 true가 된다.|
|12||is_visualize_mppi|true|MPPI 과정을 시각화할지 여부이다.|
|13||constant_speed_mode|false|true일 경우 속도를 일정하게 유지한다. false일 경우 reference_speed는 waypoint에 의해 결정된다. 이때 reference_speed로 고정된다.|
|14||reference_spped|1.5|고정 속도로 사용할 경우의 참조 속도|
|15||collision_rate_threshold|1.1|충돌률이 이 값보다 크면 로봇의 속도를 0으로 설정한다.|
|16|stuck detection params(멈추거나 더 이상 원하는 방향으로 진행할 수 없는 상태를 감지하는 데 사용되는 매개변수)|speed_deque_size|10|속도 정보를 저장할 덱의 크기|
|17||stuck_speed_threshold|0.3|정체를 감지하기 위한 속도 임계값|
|18||steer_1st_delay|0.1|[s] 스티어링 제어의 1차 지연 시간|
|19||mpc_mode|svg_mppi|MPPI 컨트롤러의 모드를 설정한다. 선택 가능한 값은 forward_mppi, reverse_mppi, sv_mpc, svg_mppi|
|20|common|thread_num|12|병렬 처리를 위한 스레드 수|
|21||prediction_step_size|15|예측 스텝 크기|
|22||prediction_interval|0.05|예측 간격|
|23||steer_delay|0.025|[s] 스티어링 지연 시간. dead time. dead time. 설정값이 충분히 크면, 예측은 Dynamic Window Approach planner의 작동 방식과 유사해진다.|
|24||max_steer_angle|0.45|최대 스티어링 각도|
|25||min_steer_angle|-0.45|최소 스티어링 각도|
|26||speed_prediction_mode|reference|속도 예측 모드. 선택 가능한 값은 linear, constant, reference|
|27||max_accel|5.0|최대 가속도|
|28||min_accel|-3.0|최소 가속도|
|29||lr|0.135|로봇의 뒷바퀴 축위치|
|30||lf|0.189|로봇의 앞바쿼 축 위치|
|31||collision_weight|1.0|충돌 가중치|
|32||q_dist|1.0|경로 최적화 시 사용되는 거리 및 각도에 대한 가중치|
|33||q_angle|0.01|경로 최적화 시용되는 거리 및 각도에 대한 가중치|
|34||q_terminal_dist|1.0|경로 최적화 사용되는 거리 및 각도에 대한 가중치|
|35||q_terminal_angle|0.01|경로 최적화 사용되는 거리 및 각도에 대한 가중치|
|36|forward_mppi|sample_batch_num|10000|샘플 배치 수|
|37||lambda|3.0|자유 에너지의 온도 매개변수, 제어 비용과 상태 비용 사이의 균형을 맞춘다.|
|38||alpha|0.1|이전 제어 시퀀스와 명목 제어 시퀀스 간의 균형을 맞추는 가중치 매개변수|
|39||non_biased_sampling_rate|0.1|무작위 노이즈를 추가할 비율|
|40||steer_cov|0.1|스티어링 공분산|
|41||num_itr_for_grad_estimation|0||
|42||step_size_for_grad_estimation|0.001||
|43||sample_num_for_grad_estimation|5||
|44||steer_cov_for_grad_estimation|0.001||
|45|reverse_mppi|sample_batch_num|200|샘플 배치 수|
|46||negative_ratio|1.0|음수 비율|
|47||is_sample_rejection|true|샘플 거부 여부|
|48||sample_inflation_ratio|2.0|샘플 인플레이션 비율||
|49||iteration_num|50|반복 횟수|
|50||step_size|0.05|스텝 크기|
|51||warm_start_ratio|0.5|워밍 스타트 비율|
|52||lambda|3.0||
|53||alpha|0.1||
|54||non_biased_sampling_rate|0.1||
|55||steer_cov|0.1||
|56|stein_variational_mpc|sample_batch_num|500|샘플 배치 수|
|57||lambda|3.0|자유 에너지의 온도 매개변수로 제어 비용과 상태 비용 간의 균형을 조절한다. 큰 값은 제어 비용에 더 많은 중요성을 부여한다.|
|58||alpha|0.1|이전 제어 시퀀스와 명목 제어 시퀀스 사이의 균형을 조절하는 가중치 매개변수이다. 이전 제어의 중요도 결정한다.|
|59||non_biased_sampling_rate|0.1|제어 시퀀스 후보에 무작위 노이즈를 추가하는 비율을 정의한다. 이는 탐색의 다양성을 증가시킨다.|
|60||steer_cov|0.1|초기 조항 공분산으로 조항 제어의 변동성을 나타낸다. 이는 제어 명령의 불확실성을 조절한다.|
|61||num_svgd_iteration|3|Stein Variational Gradient Descent의 반복 횟수를 나타낸다. 이는 샘플 집합의 최적화 깊이를 결정한다.|
|62||sample_num_for_grad_estimation|10|그래디언트 추정을 위한 샘플 수이다. 그래디언트 추정의 정확성을 높이는 데 사용된다.|
|63||steer_cov_for_grad_estimation|0.01|그래디언트 추정 시 사용되는 조항 공분산이다. 추정의 세밀도를 조절한다.|
|64||svgd_step_size|0.5|SVGD 알고리즘의 단계 크기로 샘플 집합을 이동시키는 크기를 나타낸다.|
|65||is_max_posterior_estimation|false|최대 사후 추정 여부를 나타낸다. false이면 SVGD를 통한 최적화에 중점을 둔다.|
|66|svg_mppi|sample_batch_num|8000|샘플 배치의 크기를 나타내며, 이는 제어 시퀀스를 평가하기 위한 샘플 수를 결정한다.|
|67||lambda|3.0|자유 에너지의 온도 매개변수로 제어 비용과 상태 비용 간의 균형을 조절한다.|
|68||non_biased_sampling_rate|0.1|제어 시퀀스 후보에 무작위 노이즈를 추가하는 비율을 정의한다.|
|69||alpha|0.1|이전 제어 시퀀스와 명목 제어 시퀀스 사이의 균형을 조절하는 가중치 매개변수|
|70||steer_cov|0.01|초기 조항 공분산으로 조항 제어의 변동성을 나타낸다.|
|71||alpha|0.1|이전 제어 시퀀스와 명목 제어 시퀀스 사이의 균형을 조절하는 가중치 매개변수이다.|
|72||steer_cov|0.01|초기 조항 공분산으로 조항 제어의 변동성을 나타낸다.|
|73||guide_sample_num|1|안내 샘플 수를 나타내며 제어 시퀀스의 최적화를 돕는다.|
|74||grad_lambda|3.0|그래디언트 추정 시의 람다 값으로 추정의 세밀도를 조절한다.|
|75||sample_num_for_grad_estimation|100|그래디언트 추정을 위한 샘플 수|
|76||steer_cov_forgrad_estimation|0.01|
|77||svgd_step_size|0.005|
|78||num_svgd_iteration|10|SVGD 반복 횟수를 나타내며 샘플 집합의 최적화 깊이를 결정한다.|
|79||is_use_nominal_solution|true|명목 솔루션 사용 여부를 나타낸다. true이면 명목 솔루션을 사용하여 초기화를 수행한다.|
|80||is_covariance_adaptation|true|공분산 적응 여부를 나타낸다. true이면 공분산을 환경에 맞춰 조정한다.|
|81|0.1|gaussian_fitting_lambda|가우시안 피팅 시 람다 값을 나타낸다.|
|82||min_steer_cov|0.001|조항 공분산의 최소값을 정의한다.|
|83||max_steer_cov|0.1|조항 공분산의 최대값을 정의한다.

# ROS1 Functions

## src/mppi_controller_ros.cpp

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

## src/mpc_base.cpp

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
|17|std::pair<StateSeq, XYCovMatrices>|MPCBase::calc_state_distribution|const StateSeqBatch& state_seq_candidates|여러 상태 시퀀스 후보들로부터 평균 상태 시퀀스를 계산하고, 각 예측 단계에서 x와 y 위치의 공분산 행렬을 계산하는 함수이다. 이 함수는 MPC에서 경로 예측의 불확실성을 평가하고, 상태 분포를 분석하기 위해 사용된다.|
