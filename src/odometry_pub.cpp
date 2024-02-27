#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"  // Pathを使用するために追加
#include "tf2/LinearMath/Quaternion.h"  // tf2::Quaternionを使用するために追加
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_pub")
  {
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("odom_path", 50);
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // cmd_velサブスクライバを追加
    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&OdometryPublisher::cmd_vel_callback, this, std::placeholders::_1));

    x = 0.0;
    y = 0.0;
    th = 0.0;

    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();

    path.header.frame_id = "odom";  // パスのフレームIDを設定

    timer_ = this->create_wall_timer(100ms, std::bind(&OdometryPublisher::timer_callback, this));

    // 静的な変換を送信するタイマー
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // cmd_velメッセージから速度を更新
    vx = msg->linear.x;
    vth = msg->angular.z;
  }

  void timer_callback()
  {
    current_time = this->get_clock()->now();

    double dt = (current_time - last_time).seconds();
    double delta_x = vx * cos(th) * dt;
    double delta_y = vx * sin(th) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th);  // ロール、ピッチ、ヨーをセット
    geometry_msgs::msg::Quaternion odom_quat_msg =
      tf2::toMsg(odom_quat);  // tf2::Quaternionからgeometry_msgs::msg::Quaternionに変換

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat_msg;

    odom_broadcaster->sendTransform(odom_trans);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat_msg;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    odom_pub->publish(odom);

    // パスに現在の位置を追加
    geometry_msgs::msg::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;
    this_pose_stamped.pose.orientation = odom_quat_msg;
    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);

    // パスを公開
    path_pub->publish(path);

    last_time = current_time;
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->get_clock()->now();
    static_transform_stamped.header.frame_id = "map";
    static_transform_stamped.child_frame_id = "odom";
    static_transform_stamped.transform.translation.x = 1.0;
    static_transform_stamped.transform.translation.y = 1.0;
    static_transform_stamped.transform.translation.z = 0.0;  // 仮にz方向に0.1mのオフセット
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(static_transform_stamped);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;  // パスを公開するためのパブリッシャー
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path;  // Pathメッセージのメンバ変数を追加
  double x, y, th, vx, vth;
  rclcpp::Time current_time, last_time;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
