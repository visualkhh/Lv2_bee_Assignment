#pragma once
// fake_nvblox — nvblox ESDF 대역 (GPU 없이 headless 테스트용).
//   실제 nvblox는 depth 영상으로 TSDF를 쌓아 장애물까지 거리를 넣은 점구름을 낸다.
//   테스트에선 지도가 비어 있다고 가정하고, 모든 점의 intensity(거리)를
//   lidar_max 이상으로 둬서 "free space"로 보이게 한다.
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace faker {

class FakeNvblox : public rclcpp::Node {
public:
  FakeNvblox();

private:
  void on_timer();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_ = "base_footprint";
  double radius_m_ = 5.0;   // 점을 찍는 반경 [m]
  int beams_ = 72;          // 점 개수
  double z_m_ = 0.1;        // 점 높이 [m]
  float intensity_m_ = 3.5;  // ESDF 거리 [m] — lidar_max 이상이면 전부 free
};

}  // namespace faker
