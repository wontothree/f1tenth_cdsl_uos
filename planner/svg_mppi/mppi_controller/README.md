# MPPI Controller Package

# ROS1 Variables and Functions

## mppi_controller_ros.cpp

|Index|Return Type|Function|Input|Description|
|---|---|---|---|---|
|1|void|MPPIControllerROS::callback_odom_with_pose|const nav_msgs::Odometry& odom|Odometry 정보(선형 속도, 각속도)를 기반으로 로봇의 위치와 자세를 추정한다.|
|2|void|MPPIControllerROS::callback_odom|const nav_msgs::Odometry& odom|Odom 정보를 입력받아 로봇의 현재 속도 정보를 업데이트하는 함수. 위치 추정 없이 로봇의 속도만을 사용하는 간단한 형태의 함수. 주로 로봇의 속도 정보만 필요로 하는 시스템에서 사용된다.|
|3|void|MPPIControllerROS::callback_activate_signal|const std_msgs::Bool& is_activate||