// Pratica 03 - Exercicio 1.1: controle de pose CONTINUO.
//
// Le a pose desejada de um topico (geometry_msgs/PoseStamped, padrao
// /goal_pose - o mesmo da ferramenta "2D Goal Pose" do RViz) e publica
// comandos em /cmd_vel.
//
// Lei de controle em coordenadas polares (Siegwart, "Introduction to
// Autonomous Mobile Robots"), com o erro de pose expresso por
//   rho   = distancia ate o alvo
//   alpha = angulo entre a frente do robo e a direcao do alvo
//   beta  = angulo entre a direcao do alvo e a orientacao final desejada
// e os comandos
//   v = PID_rho(rho) * max(cos(alpha), 0)
//   w = PID_alpha(alpha) + PID_beta(beta)
// Estabilidade (caso P puro): k_rho > 0, k_beta < 0, k_alpha > k_rho.
// O fator cos(alpha) zera o avanco enquanto o alvo esta atras do robo, para ele
// primeiro virar em vez de se afastar em curva.
//
// Com rho abaixo da tolerancia, o robo so corrige a orientacao final
// (PID_yaw) e, ao atingir a tolerancia angular, publica true em goal_reached.
//
// Os ganhos vem de config/pose_controller.yaml (control_toolbox::Pid).

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_trajectory/common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
namespace mrt = my_robot_trajectory;

class PoseController : public rclcpp::Node
{
public:
  PoseController()
  : Node("pose_controller")
  {
    pos_tol_ = declare_parameter("position_tolerance", 0.05);
    yaw_tol_ = declare_parameter("yaw_tolerance", 0.05);
    v_max_ = declare_parameter("max_linear_velocity", 0.8);
    w_max_ = declare_parameter("max_angular_velocity", 1.5);

    pid_rho_ = mrt::declarePid(*this, "pid_rho", 0.5);
    pid_alpha_ = mrt::declarePid(*this, "pid_alpha", 1.5);
    pid_beta_ = mrt::declarePid(*this, "pid_beta", -0.3);
    pid_yaw_ = mrt::declarePid(*this, "pid_yaw", 1.5);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // transient_local: quem se inscrever depois (ex. mission_follower) ainda
    // recebe o ultimo estado.
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

    RCLCPP_INFO(get_logger(), "Controle de pose continuo pronto. Aguardando alvo em %s",
      goal_sub_->get_topic_name());
  }

private:
  enum class Phase { IDLE, DRIVE, ALIGN };

  void onGoal(const geometry_msgs::msg::PoseStamped & m)
  {
    goal_.x = m.pose.position.x;
    goal_.y = m.pose.position.y;
    goal_.yaw = mrt::yawFromQuaternion(m.pose.orientation);
    pid_rho_.reset();
    pid_alpha_.reset();
    pid_beta_.reset();
    pid_yaw_.reset();
    phase_ = Phase::DRIVE;
    std_msgs::msg::Bool b;
    b.data = false;
    reached_pub_->publish(b);
    RCLCPP_INFO(get_logger(), "Novo alvo: x=%.2f y=%.2f yaw=%.1f deg",
      goal_.x, goal_.y, goal_.yaw * 180.0 / M_PI);
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
    const double rho = std::hypot(dx, dy);
    const double yaw_err = mrt::wrapAngle(goal_.yaw - pose_.yaw);

    // Histerese: so volta a transladar se o robo sair bem da tolerancia.
    if (phase_ == Phase::DRIVE && rho < pos_tol_) {
      phase_ = Phase::ALIGN;
      pid_yaw_.reset();
    } else if (phase_ == Phase::ALIGN && rho > 3.0 * pos_tol_) {
      phase_ = Phase::DRIVE;
    }

    double v = 0.0;
    double w = 0.0;
    if (phase_ == Phase::DRIVE) {
      const double path_dir = std::atan2(dy, dx);
      const double alpha = mrt::wrapAngle(path_dir - pose_.yaw);
      const double beta = mrt::wrapAngle(goal_.yaw - path_dir);
      v = pid_rho_.computeCommand(rho, dt) * std::max(std::cos(alpha), 0.0);
      w = pid_alpha_.computeCommand(alpha, dt) + pid_beta_.computeCommand(beta, dt);
    } else {  // ALIGN
      if (std::abs(yaw_err) < yaw_tol_) {
        cmd_pub_->publish(mrt::makeTwist(0.0, 0.0));
        phase_ = Phase::IDLE;
        std_msgs::msg::Bool b;
        b.data = true;
        reached_pub_->publish(b);
        RCLCPP_INFO(get_logger(), "Alvo atingido (erro pos=%.3f m, yaw=%.2f deg)",
          rho, yaw_err * 180.0 / M_PI);
        return;
      }
      w = pid_yaw_.computeCommand(yaw_err, dt);
    }

    mrt::scaleToLimits(v, w, v_max_, w_max_);
    cmd_pub_->publish(mrt::makeTwist(v, w));
  }

  double pos_tol_, yaw_tol_, v_max_, w_max_;
  control_toolbox::Pid pid_rho_, pid_alpha_, pid_beta_, pid_yaw_;

  Phase phase_{Phase::IDLE};
  mrt::Pose2D pose_, goal_;
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
  rclcpp::spin(std::make_shared<PoseController>());
  rclcpp::shutdown();
  return 0;
}
