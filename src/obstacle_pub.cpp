#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct Obstacle
{
  double x, y;    // 障害物の中心座標 (メートル)
  double radius;  // 障害物の半径 (メートル)
  double margin;  // マージン (メートル)
};

class ObstaclePublisher : public rclcpp::Node
{
public:
  ObstaclePublisher()
  : Node("obstacle_pub")
  {
    marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ObstaclePublisher::obstacle_publish, this));

    // 障害物のデータをobstaclesに追加
    obstacles.push_back({2.0, 0.0, 0.5, 0.2});    // 1つ目の障害物
    obstacles.push_back({2.0, 2.0, 0.5, 0.2});    // 2つ目の障害物
    obstacles.push_back({2.0, -2.0, 0.5, 0.2});   // 3つ目の障害物
    obstacles.push_back({0.0, 2.0, 0.5, 0.2});    // 4つ目の障害物
    obstacles.push_back({0.0, -2.0, 0.5, 0.2});   // 5つ目の障害物
    obstacles.push_back({-2.0, 0.0, 0.5, 0.2});   // 6つ目の障害物
    obstacles.push_back({-2.0, 2.0, 0.5, 0.2});   // 7つ目の障害物
    obstacles.push_back({-2.0, -2.0, 0.5, 0.2});  // 8つ目の障害物
    obstacles.push_back({4.0, 4.0, 0.5, 0.2});    // 9つ目の障害物
    obstacles.push_back({4.0, 1.0, 0.5, 0.2});    // 10つ目の障害物
    obstacles.push_back({4.0, -1.0, 0.5, 0.2});   // 11つ目の障害物
    obstacles.push_back({4.0, -4.0, 0.5, 0.2});   // 12つ目の障害物
    obstacles.push_back({-4.0, 4.0, 0.5, 0.2});   // 13つ目の障害物
    obstacles.push_back({-4.0, 1.0, 0.5, 0.2});   // 14つ目の障害物
    obstacles.push_back({-4.0, -1.0, 0.5, 0.2});  // 15つ目の障害物
    obstacles.push_back({-4.0, -4.0, 0.5, 0.2});  // 16つ目の障害物
    obstacles.push_back({1.0, 4.0, 0.5, 0.2});   // 17つ目の障害物
    obstacles.push_back({-1.0, 4.0, 0.5, 0.2});   // 18つ目の障害物
    obstacles.push_back({1.0, -4.0, 0.5, 0.2});  // 19つ目の障害物
    obstacles.push_back({-1.0, -4.0, 0.5, 0.2});  // 20つ目の障害物
  }

private:
  void obstacle_publish()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto & obstacle : obstacles) {
      // 障害物の中心位置と半径
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Node::now();
      marker.ns = "obstacle_clusters";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacle.x;
      marker.pose.position.y = obstacle.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle.radius;
      marker.scale.y = obstacle.radius;
      marker.scale.z = 0.5;   // Height of the cylinder
      marker.color.a = 1.0;  // 透明度
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;

      marker_array.markers.push_back(marker);
    }

    marker_publisher_->publish(marker_array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Obstacle> obstacles;  // 障害物のリスト
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}
