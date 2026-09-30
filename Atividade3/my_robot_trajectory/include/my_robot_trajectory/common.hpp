// Utilitarios compartilhados pelos nos da pratica 03.
#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "control_toolbox/pid.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_trajectory
{

// Normaliza um angulo para o intervalo (-pi, pi].
inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Guinada (yaw) de um quaternion, para robos que so giram em torno de Z.
inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

// Declara os parametros <name>.kp/ki/kd/i_max/i_min/antiwindup e devolve um
// control_toolbox::Pid com esses ganhos. Os valores reais vem do YAML; os
// argumentos sao so o padrao caso o arquivo nao defina o ganho.
inline control_toolbox::Pid declarePid(
  rclcpp::Node & node, const std::string & name,
  double kp, double ki = 0.0, double kd = 0.0, double i_max = 0.0)
{
  const double p = node.declare_parameter(name + ".kp", kp);
  const double i = node.declare_parameter(name + ".ki", ki);
  const double d = node.declare_parameter(name + ".kd", kd);
  const double imax = node.declare_parameter(name + ".i_max", i_max);
  // sem i_min no YAML, o limite e simetrico ao i_max lido
  const double imin = node.declare_parameter(name + ".i_min", -imax);
  const bool antiwindup = node.declare_parameter(name + ".antiwindup", true);
  RCLCPP_INFO(
    node.get_logger(), "PID %-10s kp=%.3f ki=%.3f kd=%.3f i_lim=[%.2f, %.2f]",
    name.c_str(), p, i, d, imin, imax);
  return control_toolbox::Pid(p, i, d, imax, imin, antiwindup);
}

// Satura (v, w) mantendo a razao entre eles: preserva a curvatura do comando,
// o que importa no seguimento de trajetoria (saturar cada eixo separadamente
// deformaria o caminho).
inline void scaleToLimits(double & v, double & w, double v_max, double w_max)
{
  const double s = std::max({1.0, std::abs(v) / v_max, std::abs(w) / w_max});
  v /= s;
  w /= s;
}

inline geometry_msgs::msg::Twist makeTwist(double v, double w)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = v;
  t.angular.z = w;
  return t;
}

// Pose planar do robo (lida do /odom).
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

}  // namespace my_robot_trajectory
