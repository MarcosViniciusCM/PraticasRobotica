// Pratica 03 - Exercicio 1.2: seguimento de missao.
//
// Carrega uma lista de waypoints (x, y, yaw) do YAML e os envia, um de cada
// vez, ao controlador de pose pelo topico /goal_pose. So passa para o proximo
// quando o controlador publica true em goal_reached. Funciona com qualquer um
// dos dois controladores (pose_controller ou three_step_controller), o que
// permite comparar os dois na mesma missao (bonus).
//
// Formato do YAML:
//   waypoints: ["p1", "p2", "p3"]
//   p1: [x, y, yaw_graus]
//
// Ao final imprime um resumo (tempo, distancia percorrida, erro final em cada
// waypoint) e, se results_file estiver definido, acrescenta uma linha CSV.

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "my_robot_trajectory/common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
namespace mrt = my_robot_trajectory;

class MissionFollower : public rclcpp::Node
{
public:
  MissionFollower()
  : Node("mission_follower")
  {
    const auto names = declare_parameter("waypoints", std::vector<std::string>{});
    for (const auto & n : names) {
      const auto v = declare_parameter(n, std::vector<double>{});
      if (v.size() != 3) {
        throw std::runtime_error("waypoint '" + n + "' deve ser [x, y, yaw_graus]");
      }
      waypoints_.push_back({n, {v[0], v[1], v[2] * M_PI / 180.0}});
    }
    if (waypoints_.size() < 3) {
      RCLCPP_WARN(get_logger(), "O roteiro pede ao menos 3 waypoints (ha %zu)",
        waypoints_.size());
    }
    if (waypoints_.empty()) {
      throw std::runtime_error("nenhum waypoint definido no parametro 'waypoints'");
    }

    frame_id_ = declare_parameter("frame_id", std::string("odom"));
    start_delay_ = declare_parameter("start_delay", 2.0);
    wp_timeout_ = declare_parameter("waypoint_timeout", 60.0);
    loop_ = declare_parameter("loop", false);
    label_ = declare_parameter("label", std::string(""));
    results_file_ = declare_parameter("results_file", std::string(""));
    shutdown_when_done_ = declare_parameter("shutdown_when_done", false);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {onOdom(*m);});

    reached_sub_ = create_subscription<std_msgs::msg::Bool>(
      "goal_reached", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr m) {onReached(m->data);});

    timer_ = rclcpp::create_timer(this, get_clock(), 100ms, [this]() {step();});

    RCLCPP_INFO(get_logger(), "Missao com %zu waypoints carregada", waypoints_.size());
    for (const auto & wp : waypoints_) {
      RCLCPP_INFO(get_logger(), "  %-6s x=%6.2f y=%6.2f yaw=%7.1f deg", wp.name.c_str(),
        wp.pose.x, wp.pose.y, wp.pose.yaw * 180.0 / M_PI);
    }
  }

private:
  struct Waypoint
  {
    std::string name;
    mrt::Pose2D pose;
  };

  struct Result
  {
    double time;
    double pos_err;
    double yaw_err;
    bool timed_out;
  };

  enum class State { WAIT_SYSTEM, WAIT_START, RUNNING, DONE };

  void onOdom(const nav_msgs::msg::Odometry & m)
  {
    mrt::Pose2D p;
    p.x = m.pose.pose.position.x;
    p.y = m.pose.pose.position.y;
    p.yaw = mrt::yawFromQuaternion(m.pose.pose.orientation);
    if (have_odom_ && state_ == State::RUNNING) {
      distance_ += std::hypot(p.x - pose_.x, p.y - pose_.y);
    }
    pose_ = p;
    have_odom_ = true;
  }

  void onReached(bool reached)
  {
    if (state_ != State::RUNNING) {
      return;
    }
    // O controlador publica false ao aceitar o alvo e true ao atingi-lo. Exigir
    // o false antes evita aceitar um true antigo (o topico e transient_local).
    if (!reached) {
      acked_ = true;
    } else if (acked_) {
      finishWaypoint(false);
    }
  }

  void sendGoal()
  {
    const auto & wp = waypoints_[idx_];
    geometry_msgs::msg::PoseStamped g;
    g.header.stamp = now();
    g.header.frame_id = frame_id_;
    g.pose.position.x = wp.pose.x;
    g.pose.position.y = wp.pose.y;
    g.pose.orientation = mrt::quaternionFromYaw(wp.pose.yaw);
    acked_ = false;
    wp_start_ = now();
    goal_pub_->publish(g);
    RCLCPP_INFO(get_logger(), "[%zu/%zu] indo para %s (%.2f, %.2f, %.1f deg)",
      idx_ + 1, waypoints_.size(), wp.name.c_str(), wp.pose.x, wp.pose.y,
      wp.pose.yaw * 180.0 / M_PI);
  }

  void finishWaypoint(bool timed_out)
  {
    const auto & wp = waypoints_[idx_];
    Result r;
    r.time = (now() - wp_start_).seconds();
    r.pos_err = std::hypot(wp.pose.x - pose_.x, wp.pose.y - pose_.y);
    r.yaw_err = mrt::wrapAngle(wp.pose.yaw - pose_.yaw);
    r.timed_out = timed_out;
    results_.push_back(r);
    if (timed_out) {
      RCLCPP_WARN(get_logger(), "%s: tempo esgotado (%.0f s), seguindo para o proximo",
        wp.name.c_str(), wp_timeout_);
    } else {
      RCLCPP_INFO(get_logger(), "%s atingido em %.2f s", wp.name.c_str(), r.time);
    }

    ++idx_;
    if (idx_ >= waypoints_.size()) {
      if (loop_) {
        idx_ = 0;
      } else {
        state_ = State::DONE;
        report();
        return;
      }
    }
    sendGoal();
  }

  void step()
  {
    switch (state_) {
      case State::WAIT_SYSTEM:
        // Espera odometria e um controlador inscrito no topico do alvo; senao
        // o primeiro waypoint seria publicado no vazio.
        if (have_odom_ && goal_pub_->get_subscription_count() > 0) {
          state_ = State::WAIT_START;
          wait_start_ = now();
          RCLCPP_INFO(get_logger(), "Controlador e odometria prontos, iniciando em %.1f s",
            start_delay_);
        }
        break;
      case State::WAIT_START:
        if ((now() - wait_start_).seconds() >= start_delay_) {
          state_ = State::RUNNING;
          mission_start_ = now();
          distance_ = 0.0;
          sendGoal();
        }
        break;
      case State::RUNNING:
        if ((now() - wp_start_).seconds() > wp_timeout_) {
          finishWaypoint(true);
        }
        break;
      case State::DONE:
        break;
    }
  }

  void report()
  {
    const double total = (now() - mission_start_).seconds();
    double sum_pos = 0.0;
    double sum_yaw = 0.0;
    RCLCPP_INFO(get_logger(), "==================== MISSAO CONCLUIDA %s", label_.c_str());
    RCLCPP_INFO(get_logger(), " waypoint   tempo[s]  erro_pos[m]  erro_yaw[deg]");
    for (size_t i = 0; i < results_.size(); ++i) {
      const auto & r = results_[i];
      RCLCPP_INFO(get_logger(), " %-8s %9.2f %12.4f %14.2f%s",
        waypoints_[i % waypoints_.size()].name.c_str(), r.time, r.pos_err,
        r.yaw_err * 180.0 / M_PI, r.timed_out ? "  (timeout)" : "");
      sum_pos += r.pos_err;
      sum_yaw += std::abs(r.yaw_err);
    }
    const double n = static_cast<double>(results_.size());
    RCLCPP_INFO(get_logger(), " tempo total      : %.2f s", total);
    RCLCPP_INFO(get_logger(), " distancia        : %.3f m", distance_);
    RCLCPP_INFO(get_logger(), " erro pos medio   : %.4f m", sum_pos / n);
    RCLCPP_INFO(get_logger(), " erro yaw medio   : %.2f deg", sum_yaw / n * 180.0 / M_PI);

    if (!results_file_.empty()) {
      std::ifstream probe(results_file_);
      const bool is_new = !probe.good() || probe.peek() == std::ifstream::traits_type::eof();
      probe.close();
      std::ofstream f(results_file_, std::ios::app);
      if (is_new) {
        f << "label,waypoints,tempo_total_s,distancia_m,erro_pos_medio_m,erro_yaw_medio_deg\n";
      }
      f << label_ << ',' << results_.size() << ',' << total << ',' << distance_ << ','
        << sum_pos / n << ',' << sum_yaw / n * 180.0 / M_PI << '\n';
      RCLCPP_INFO(get_logger(), "Resultado acrescentado em %s", results_file_.c_str());
    }

    if (shutdown_when_done_) {
      rclcpp::shutdown();
    }
  }

  std::vector<Waypoint> waypoints_;
  std::vector<Result> results_;
  std::string frame_id_, label_, results_file_;
  double start_delay_, wp_timeout_;
  bool loop_, shutdown_when_done_;

  State state_{State::WAIT_SYSTEM};
  size_t idx_{0};
  bool acked_{false};
  bool have_odom_{false};
  mrt::Pose2D pose_;
  double distance_{0.0};
  rclcpp::Time wait_start_, mission_start_, wp_start_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionFollower>());
  rclcpp::shutdown();
  return 0;
}
