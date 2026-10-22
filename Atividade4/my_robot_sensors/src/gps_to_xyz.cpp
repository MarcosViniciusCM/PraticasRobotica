#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpsToXyz : public rclcpp::Node
{
public:
  GpsToXyz()
  : Node("gps_to_xyz"),
    datum_set_(false)
  {
    use_first_fix_ = declare_parameter("use_first_fix_as_datum", true);
    datum_lat_ = declare_parameter("datum_lat", 0.0);
    datum_lon_ = declare_parameter("datum_lon", 0.0);
    datum_alt_ = declare_parameter("datum_alt", 0.0);
    frame_id_ = declare_parameter("frame_id", std::string("odom"));

    if (!use_first_fix_) {
      datum_set_ = true;
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gps/pose", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("gps/odom", 10);

    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps/fix", 10,
      std::bind(&GpsToXyz::onFix, this, std::placeholders::_1));
  }

private:
  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) {
      return;
    }

    // Define a primeira coordenada valida recebida como o referencial local (datum)
    if (!datum_set_) {
      datum_lat_ = msg->latitude;
      datum_lon_ = msg->longitude;
      datum_alt_ = msg->altitude;
      datum_set_ = true;
    }

    // Projecao equirretangular local de coordenadas geograficas para plano cartesiano (metros)
    constexpr double earth_radius = 6378137.0;
    constexpr double deg_to_rad = M_PI / 180.0;

    const double d_lat = (msg->latitude - datum_lat_) * deg_to_rad;
    const double d_lon = (msg->longitude - datum_lon_) * deg_to_rad;
    const double lat_ref = datum_lat_ * deg_to_rad;

    const double x = earth_radius * d_lon * std::cos(lat_ref);
    const double y = earth_radius * d_lat;
    const double z = msg->altitude - datum_alt_;

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;
    pose_msg.pose.orientation.w = 1.0;
    pose_pub_->publish(pose_msg);

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose = pose_msg.pose;
    odom_pub_->publish(odom_msg);
  }

  bool use_first_fix_;
  bool datum_set_;
  double datum_lat_;
  double datum_lon_;
  double datum_alt_;
  std::string frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsToXyz>());
  rclcpp::shutdown();
  return 0;
}
