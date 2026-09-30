// Pratica 03 - Exercicio 1.1 (bonus): controle de pose com TRES MANOBRAS.
//
// Mesma interface do pose_controller (le /goal_pose, publica /cmd_vel e
// goal_reached), mas o movimento e decomposto em tres etapas desacopladas:
//
//   1. ROTATE_TO_GOAL - gira no lugar ate apontar para o alvo
//                       w = PID_angular(atan2(dy, dx) - yaw),  v = 0
//   2. TRANSLATE      - anda em linha reta ate o alvo
//                       v = PID_linear(d),  w = PID_heading(phi - yaw)
//                       d   = distancia ate o alvo projetada na direcao phi
//                             (com sinal: negativa se passou do ponto)
//                       phi = direcao da reta, congelada no inicio da etapa
//   3. ROTATE_TO_YAW  - gira no lugar ate a orientacao final
//                       w = PID_angular(yaw_goal - yaw),  v = 0
//
// Os ganhos vem de config/three_step_controller.yaml (control_toolbox::Pid).

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_trajectory/common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace mrt = my_robot_trajectory;

class ThreeStepController : public rclcpp::Node
{
public:
  ThreeStepController()
  : Node("three_step_controller")
  {
    pos_tol_ = declare_parameter("position_tolerance", 0.05);
    yaw_tol_ = declare_parameter("yaw_tolerance", 0.05);
    v_max_ = declare_parameter("max_linear_velocity", 0.8);
    w_max_ = declare_parameter("max_angular_velocity", 1.5);

    pid_angular_ = mrt::declarePid(*this, "pid_angular", 2.0);
    pid_linear_ = mrt::declarePid(*this, "pid_linear", 0.8);
    pid_heading_ = mrt::declarePid(*this, "pid_heading", 2.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>(
      "goal_reached", rclcpp::QoS(1).transient_local());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        pose_.x = m->pose.pose.position.x;
        pose_.y = m->pose.pose.position.y;
        pose_.yaw = mrt::yawFromQuaternion(m->pose.pose.orientation);
        have_odom_ = true;
        // controle disparado pela odometria (50 Hz), com o carimbo dela como
        // relogio - ver figure8_tracker.cpp
        step(rclcpp::Time(m->header.stamp, RCL_ROS_TIME));
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        onGoal(*m);
      });

    RCLCPP_INFO(get_logger(), "Controle de pose em 3 manobras pronto. Aguardando alvo em %s",
      goal_sub_->get_topic_name());
  }

private:
  enum class Phase { IDLE, ROTATE_TO_GOAL, TRANSLATE, ROTATE_TO_YAW };

  static const char * name(Phase p)
  {
    switch (p) {
      case Phase::ROTATE_TO_GOAL: return "rotacao 1";
      case Phase::TRANSLATE: return "translacao";
      case Phase::ROTATE_TO_YAW: return "rotacao 2";
      default: return "parado";
    }
  }

  void setPhase(Phase p)
  {
    phase_ = p;
    pid_angular_.reset();
    pid_linear_.reset();
    pid_heading_.reset();
    if (p == Phase::TRANSLATE) {
      line_dir_ = std::atan2(goal_.y - pose_.y, goal_.x - pose_.x);
    }
    RCLCPP_INFO(get_logger(), "Manobra: %s", name(p));
  }

  void onGoal(const geometry_msgs::msg::PoseStamped & m)
  {
    goal_.x = m.pose.position.x;
    goal_.y = m.pose.position.y;
    goal_.yaw = mrt::yawFromQuaternion(m.pose.orientation);
    std_msgs::msg::Bool b;
    b.data = false;
    reached_pub_->publish(b);
    RCLCPP_INFO(get_logger(), "Novo alvo: x=%.2f y=%.2f yaw=%.1f deg",
      goal_.x, goal_.y, goal_.yaw * 180.0 / M_PI);
    if (!have_odom_ || std::hypot(goal_.x - pose_.x, goal_.y - pose_.y) > pos_tol_) {
      setPhase(Phase::ROTATE_TO_GOAL);
    } else {
      setPhase(Phase::ROTATE_TO_YAW);
    }
  }

  void step(const rclcpp::Time & t)
  {
    if (phase_ == Phase::IDLE || last_time_.nanoseconds() == 0) {
      last_time_ = t;
      return;
    }
    const int64_t dt = (t - last_time_).nanoseconds();
    if (dt <= 0) {
      return;
    }
    last_time_ = t;

    const double dx = goal_.x - pose_.x;
    const double dy = goal_.y - pose_.y;
    double v = 0.0;
    double w = 0.0;

    switch (phase_) {
      case Phase::ROTATE_TO_GOAL: {
          const double e = mrt::wrapAngle(std::atan2(dy, dx) - pose_.yaw);
          if (std::abs(e) < yaw_tol_) {
            setPhase(Phase::TRANSLATE);
            return;
          }
          w = pid_angular_.computeCommand(e, dt);
          break;
        }
      case Phase::TRANSLATE: {
          const double d = dx * std::cos(line_dir_) + dy * std::sin(line_dir_);
          if (std::abs(d) < pos_tol_) {
            setPhase(Phase::ROTATE_TO_YAW);
            return;
          }
          v = pid_linear_.computeCommand(d, dt);
          w = pid_heading_.computeCommand(mrt::wrapAngle(line_dir_ - pose_.yaw), dt);
          break;
        }
      case Phase::ROTATE_TO_YAW: {
          const double e = mrt::wrapAngle(goal_.yaw - pose_.yaw);
          if (std::abs(e) < yaw_tol_) {
            cmd_pub_->publish(mrt::makeTwist(0.0, 0.0));
            phase_ = Phase::IDLE;
            std_msgs::msg::Bool b;
            b.data = true;
            reached_pub_->publish(b);
            RCLCPP_INFO(get_logger(), "Alvo atingido (erro pos=%.3f m, yaw=%.2f deg)",
              std::hypot(dx, dy), e * 180.0 / M_PI);
            return;
          }
          w = pid_angular_.computeCommand(e, dt);
          break;
        }
      default:
        return;
    }

    v = std::clamp(v, -v_max_, v_max_);
    w = std::clamp(w, -w_max_, w_max_);
    cmd_pub_->publish(mrt::makeTwist(v, w));
  }

  double pos_tol_, yaw_tol_, v_max_, w_max_;
  control_toolbox::Pid pid_angular_, pid_linear_, pid_heading_;

  Phase phase_{Phase::IDLE};
  mrt::Pose2D pose_, goal_;
  double line_dir_{0.0};
  bool have_odom_{false};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThreeStepController>());
  rclcpp::shutdown();
  return 0;
}
