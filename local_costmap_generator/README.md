# [Package] local_costmap_generator

# Dependencies

laser_geometry

```bash
sudo apt-get update
sudo apt-get install ros-foxy-laser-geometry
```

PCL

```bash
sudo apt update
sudo apt install libpcl-dev
sudo apt-get install ros-foxy-pcl-conversions
sudo apt install ros-foxy-pcl-ros
```

geometry-msgs

```bash
sudo apt-get install ros-foxy-geometry-msgs
```

tf2_ros

```bash
sudo apt-get update
sudo apt-get install ros-foxy-tf2-ros
sudo apt-get install ros-foxy-tf2-eigen
```

grid_map

```bash
sudo apt-get install ros-foxy-grid-map
```

OpenMP

```bash
gcc --version
sudo apt update
sudo apt install build-essential
```


# Functions

|Return Type|Function Name|Input|Description|Dependencies|
|---|---|---|---|---|
||LocalCostmapGenerator||생성자 함수||
|void|scan_callback|const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan_msg|LaserScan message를 subscribe할 때마다 호출되는 함수||
|void|timer_callback||일정 시간마다 호출되는 함수||
|||||
|void|laserscan_to_pointcloud2|const sensor_msgs::msg::LaserScan::ConstSharedPtr laserscan, sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2|sensor_msgs::LaserScan을 sensor_msgs::PointCloud2로 변환한다.||
|void|pointcloud2_to_pcl|const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2, pcl::PointCloud<pcl::PointXYZ >::Ptr pcl|sensor_msgs::PointCloud2를 pcl::PointCloud로 변환한다.|pcl_conversions, pcl_ros|
|void|preprocess_pcl|pcl::PointCloud<pcl::PointXYZ >::Ptr pcl||
|void|sensor_frame_to_robot_frame|const std::string& sensor_frame_id, const std::string& robot_frame_id, const pcl::PointCloud<pcl::PointXYZ >::ConstPtr& pcl_sensor_frame, pcl::PointCloud<pcl::PointXYZ >::Ptr& pcl_robot_frame|센서 프레임 좌표계를 로봇 프레임 좌표계로 변환한다.|geometry_msgs, tf2_ros, rclcpp, pcl_ros, Eigen|
|void|remove_pcl_within_robot||||
|void|pcl_to_costmap||||
|void|inflate_rigidbody||||

# Variables

|Data Type|Variable Name|Description|
|---|---|---|
|std::string|laserscan_topic|라이다 센서의 스캔 데이터를 수신하기 위해 토픽 이름을 저장다.|
|rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr|sub_laserscan_|토픽을 subscribe하기 위한 subscriber 객체를 나타내는 스마트 포인터|
|bool|is_laserscan_received_|LaserScan data 수신 여부|
|rclcpp::TimerBase::SharedPtr|timer_|주어진 주기마다 지정된 콜백함수를 호출하는 타이머|
|std::shared_ptr<laser_geometry::LaserProjection>|laser_projection_|laser_geometry 라이브러리에 있는 LaserProjection 클래스의 인스턴스|
|std::shared_ptr<sensor_msgs::msg::PointCloud2>|pointcloud2_|point cloud 데이터를 저장하는 스마트 포인터|
|rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr|pub_pointcloud2_|PointCloud2 Publisher|
|pcl::PointCloud<pcl::PointXYZ >::Ptr|pcl_|PCL instance|
|pcl::PointCloud<pcl::PointXYZ >::Ptr|pcl_preprocessed_|전처리된 pcl 데이터를 저장한다.|
|std::string|robot_frame_id_||
|std::string|sensor_frame_id_||

# Convention

- 멤버 변수는 뒤에 '_'를 붙이고 매개변수는 붙이지 말자.

멤버 변수 vs 로컬 변수

관련된 변수와 함수를 (+ 의존성)을 하나로 묶어서 표시하는 게 맞을까?
