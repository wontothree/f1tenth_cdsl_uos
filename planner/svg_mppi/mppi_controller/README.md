# MPPI Controller Package

# ROS1 Variables and Functions

## mppi_controller_ros.cpp

|Index|Return Type|Function|Input|Description|
|---|---|---|---|---|
|1||MPPIControllerROS::MPPIControllerROS|ROS에서 MPPI 컨트롤러의 노드를 초기화하는 생성자 함수다. 이 함수는 노드가 시작될 때 여러 파라미터를 로드하고 다양한 ROS 컴포넌트를 설정하며 필요한 데이터 스트림을 구독하고 게시할 준비를 한다.|
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