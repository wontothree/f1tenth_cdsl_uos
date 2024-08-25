# [Package] local_costmap_generator

# Dependencies

laser_geometry

```bash
sudo apt-get update
sudo apt-get install ros-foxy-laser-geometry
```

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
||||
