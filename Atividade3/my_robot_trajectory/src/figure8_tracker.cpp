// Pratica 03 - Exercicio 2.1: seguimento da trajetoria em 8.
//
// Trajetoria de referencia (lemniscata de Gerono, centrada na pose inicial):
//   x_d(t) = A sin(W t)
//   y_d(t) = B sin(W t) cos(W t) = (B/2) sin(2 W t)
// com periodo T = 2 pi / W. Derivadas analiticas:
//   dx = A W cos(W t)          ddx = -A W^2 sin(W t)
//   dy = B W cos(2 W t)        ddy = -2 B W^2 sin(2 W t)
//   theta_d = atan2(dy, dx)
//   v_d     = sqrt(dx^2 + dy^2)
//   w_d     = (dx ddy - dy ddx) / (dx^2 + dy^2)
//
// Modos (parametro "mode"):
//
//  open_loop - so aplica (v_d, w_d). Nenhuma realimentacao: qualquer erro
//              (condicao inicial, atraso, derrapagem) se acumula.
//
//  feedback  - linearizacao por realimentacao de um ponto P a uma distancia d a
//              frente do eixo das rodas. A cinematica desse ponto e inversivel:
//                [dPx]   [cos th  -d sin th] [v]
//                [dPy] = [sin th   d cos th] [w]
//              entao, escolhendo a velocidade desejada do ponto
//                u = Kff * dP_d + PID(P_d - P)       (por eixo, x e y)
//              os comandos sao
//                v =  cos th u_x + sin th u_y
//                w = (-sin th u_x + cos th u_y) / d
//              P_d e a posicao que o ponto P teria se o robo estivesse
//              exatamente sobre a trajetoria com a orientacao theta_d, de modo
//              que erro zero em P implica o centro do robo sobre o 8.
//              Kff = 0 -> so realimentacao (P puro); Kff = 1 -> realimentacao
//              + feedforward completo.
//
// Publica, para o PlotJuggler/RViz:
//   /figure8/desired_pose  (PoseStamped)  pose de referencia no instante atual
//   /figure8/desired_path  (Path)         o 8 completo (latched)
//   /figure8/error         (Vector3Stamped) x/y = erro de posicao, z = erro de
//                                           orientacao [rad]
//   /figure8/rmse          (Vector3Stamped) x = RMSE posicao [m],
//                                           y = RMSE orientacao [rad], z = t [s]
//
// Ao completar "laps" voltas, para o robo e imprime o RMSE.

#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "my_robot_trajectory/common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mrt = my_robot_trajectory;

class Figure8Tracker : public rclcpp::Node
{
public:
  Figure8Tracker()
  : Node("figure8_tracker")
  {
    A_ = declare_parameter("A", 1.0);
    B_ = declare_parameter("B", 2.0);
    W_ = declare_parameter("Omega", 0.5);
    laps_ = declare_parameter("laps", 2.0);
    mode_ = declare_parameter("mode", std::string("feedback"));
    kff_ = declare_parameter("kff", 1.0);
    d_ = declare_parameter("d", 0.3);
    v_max_ = declare_parameter("max_linear_velocity", 1.5);
    w_max_ = declare_parameter("max_angular_velocity", 4.0);
    align_start_ = declare_parameter("align_start", true);
    align_tol_ = declare_parameter("align_tolerance", 0.02);
    start_delay_ = declare_parameter("start_delay", 2.0);
    frame_id_ = declare_parameter("frame_id", std::string("odom"));
    label_ = declare_parameter("label", std::string(""));
    results_file_ = declare_parameter("results_file", std::string(""));
    shutdown_when_done_ = declare_parameter("shutdown_when_done", false);

    if (mode_ != "feedback" && mode_ != "open_loop") {
      throw std::runtime_error("mode deve ser 'feedback' ou 'open_loop' (recebido '" +
              mode_ + "')");
    }
    if (d_ <= 0.0) {
      throw std::runtime_error("d deve ser > 0");
    }

    pid_x_ = mrt::declarePid(*this, "pid_x", 1.5);
    pid_y_ = mrt::declarePid(*this, "pid_y", 1.5);
    pid_align_ = mrt::declarePid(*this, "pid_align", 2.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    desired_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "figure8/desired_pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "figure8/desired_path", rclcpp::QoS(1).transient_local());
    error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("figure8/error", 10);
    rmse_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("figure8/rmse", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        pose_.x = m->pose.pose.position.x;
        pose_.y = m->pose.pose.position.y;
        pose_.yaw = mrt::yawFromQuaternion(m->pose.pose.orientation);
        have_odom_ = true;
        // O laco de controle roda a cada odometria (50 Hz, publish_rate do
        // diff_drive_controller) e usa o carimbo dela como relogio. Um timer em
        // tempo simulado so dispara quando o /clock avanca, e o Gazebo publica
        // o /clock numa taxa baixa - o controle caia para ~20 Hz.
        step(rclcpp::Time(m->header.stamp, RCL_ROS_TIME));
      });

    RCLCPP_INFO(get_logger(),
      "Trajetoria em 8: A=%.2f B=%.2f Omega=%.3f rad/s (T=%.2f s), %.1f voltas, modo=%s, "
      "Kff=%.2f, d=%.2f", A_, B_, W_, 2.0 * M_PI / W_, laps_, mode_.c_str(), kff_, d_);
    const double v0 = std::hypot(A_ * W_, B_ * W_);
    if (v0 > v_max_) {
      RCLCPP_WARN(get_logger(),
        "A velocidade de pico da referencia (%.2f m/s) excede max_linear_velocity (%.2f)",
        v0, v_max_);
    }
  }

private:
  enum class State { WAIT_ODOM, WAIT_START, ALIGN, TRACK, DONE };

  struct Ref
  {
    double x, y, dx, dy, ddx, ddy;
  };

  // Referencia no instante t, relativa a origem (x0_, y0_) da trajetoria.
  Ref reference(double t) const
  {
    const double s1 = std::sin(W_ * t), c1 = std::cos(W_ * t);
    const double s2 = std::sin(2.0 * W_ * t), c2 = std::cos(2.0 * W_ * t);
    Ref r;
    r.x = x0_ + A_ * s1;
    r.y = y0_ + 0.5 * B_ * s2;
    r.dx = A_ * W_ * c1;
    r.dy = B_ * W_ * c2;
    r.ddx = -A_ * W_ * W_ * s1;
    r.ddy = -2.0 * B_ * W_ * W_ * s2;
    return r;
  }

  void publishPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = now();
    const int n = 400;
    for (int i = 0; i <= n; ++i) {
      const double t = (2.0 * M_PI / W_) * i / n;
      const Ref r = reference(t);
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = r.x;
      p.pose.position.y = r.y;
      p.pose.orientation = mrt::quaternionFromYaw(std::atan2(r.dy, r.dx));
      path.poses.push_back(p);
    }
    path_pub_->publish(path);
  }

  void step(const rclcpp::Time & now_t)
  {
    const int64_t dt = last_time_.nanoseconds() > 0 ?
      (now_t - last_time_).nanoseconds() : 0;
    last_time_ = now_t;

    switch (state_) {
      case State::WAIT_ODOM:
        if (have_odom_) {
          // O 8 comeca onde o robo esta: (x0, y0) = posicao inicial.
          x0_ = pose_.x;
          y0_ = pose_.y;
          publishPath();
          state_ = State::WAIT_START;
          phase_start_ = now_t;
          RCLCPP_INFO(get_logger(), "Odometria recebida, origem do 8 em (%.2f, %.2f)",
            x0_, y0_);
        }
        return;

      case State::WAIT_START:
        if ((now_t - phase_start_).seconds() >= start_delay_) {
          state_ = align_start_ ? State::ALIGN : State::TRACK;
          phase_start_ = now_t;
          if (align_start_) {
            RCLCPP_INFO(get_logger(), "Alinhando com a direcao inicial da trajetoria");
          } else {
            startTracking(now_t);
          }
        }
        return;

      case State::ALIGN: {
          // Gira no lugar ate a orientacao inicial da referencia, para que o
          // modo malha aberta comece com a mesma condicao inicial do realimentado.
          const Ref r = reference(0.0);
          const double e = mrt::wrapAngle(std::atan2(r.dy, r.dx) - pose_.yaw);
          if (std::abs(e) < align_tol_) {
            cmd_pub_->publish(mrt::makeTwist(0.0, 0.0));
            startTracking(now_t);
            return;
          }
          if (dt > 0) {
            const double w = std::clamp(pid_align_.computeCommand(e, dt), -w_max_, w_max_);
            cmd_pub_->publish(mrt::makeTwist(0.0, w));
          }
          return;
        }

      case State::TRACK:
        track(now_t, dt);
        return;

      case State::DONE:
        return;
    }
  }

  void startTracking(const rclcpp::Time & t)
  {
    state_ = State::TRACK;
    track_start_ = t;
    pid_x_.reset();
    pid_y_.reset();
    sum_sq_pos_ = sum_sq_yaw_ = max_pos_err_ = 0.0;
    n_samples_ = 0;
    RCLCPP_INFO(get_logger(), "Iniciando seguimento (%s)", mode_.c_str());
  }

  void track(const rclcpp::Time & now_t, int64_t dt)
  {
    const double t = (now_t - track_start_).seconds();
    const double t_end = laps_ * 2.0 * M_PI / W_;
    if (laps_ > 0.0 && t >= t_end) {
      finish();
      return;
    }

    const Ref r = reference(t);
    const double speed2 = r.dx * r.dx + r.dy * r.dy;
    const double th_d = std::atan2(r.dy, r.dx);
    const double v_d = std::sqrt(speed2);
    const double w_d = speed2 > 1e-9 ? (r.dx * r.ddy - r.dy * r.ddx) / speed2 : 0.0;

    double v = 0.0;
    double w = 0.0;
    if (mode_ == "open_loop") {
      v = v_d;
      w = w_d;
    } else {
      const double th = pose_.yaw;
      // ponto de controle real e desejado, a distancia d a frente do eixo
      const double px = pose_.x + d_ * std::cos(th);
      const double py = pose_.y + d_ * std::sin(th);
      const double pdx = r.x + d_ * std::cos(th_d);
      const double pdy = r.y + d_ * std::sin(th_d);
      // velocidade do ponto desejado (derivada de P_d)
      const double vpdx = r.dx - d_ * std::sin(th_d) * w_d;
      const double vpdy = r.dy + d_ * std::cos(th_d) * w_d;

      double ux = kff_ * vpdx;
      double uy = kff_ * vpdy;
      if (dt > 0) {
        ux += pid_x_.computeCommand(pdx - px, dt);
        uy += pid_y_.computeCommand(pdy - py, dt);
      }
      v = std::cos(th) * ux + std::sin(th) * uy;
      w = (-std::sin(th) * ux + std::cos(th) * uy) / d_;
    }
    mrt::scaleToLimits(v, w, v_max_, w_max_);
    cmd_pub_->publish(mrt::makeTwist(v, w));

    // ---------------- erro e RMSE (centro do robo x referencia)
    const double ex = r.x - pose_.x;
    const double ey = r.y - pose_.y;
    const double eth = mrt::wrapAngle(th_d - pose_.yaw);
    const double epos = std::hypot(ex, ey);
    sum_sq_pos_ += epos * epos;
    sum_sq_yaw_ += eth * eth;
    max_pos_err_ = std::max(max_pos_err_, epos);
    ++n_samples_;

    geometry_msgs::msg::PoseStamped des;
    des.header.stamp = now_t;
    des.header.frame_id = frame_id_;
    des.pose.position.x = r.x;
    des.pose.position.y = r.y;
    des.pose.orientation = mrt::quaternionFromYaw(th_d);
    desired_pub_->publish(des);

    geometry_msgs::msg::Vector3Stamped err;
    err.header = des.header;
    err.vector.x = ex;
    err.vector.y = ey;
    err.vector.z = eth;
    error_pub_->publish(err);

    geometry_msgs::msg::Vector3Stamped rm;
    rm.header = des.header;
    rm.vector.x = rmsePos();
    rm.vector.y = rmseYaw();
    rm.vector.z = t;
    rmse_pub_->publish(rm);
  }

  double rmsePos() const {return n_samples_ ? std::sqrt(sum_sq_pos_ / n_samples_) : 0.0;}
  double rmseYaw() const {return n_samples_ ? std::sqrt(sum_sq_yaw_ / n_samples_) : 0.0;}

  void finish()
  {
    state_ = State::DONE;
    cmd_pub_->publish(mrt::makeTwist(0.0, 0.0));
    RCLCPP_INFO(get_logger(), "==================== TRAJETORIA CONCLUIDA %s", label_.c_str());
    RCLCPP_INFO(get_logger(), " modo=%s Kff=%.2f Omega=%.3f voltas=%.1f amostras=%zu",
      mode_.c_str(), kff_, W_, laps_, n_samples_);
    RCLCPP_INFO(get_logger(), " RMSE posicao   : %.4f m", rmsePos());
    RCLCPP_INFO(get_logger(), " RMSE orientacao: %.4f rad (%.2f deg)",
      rmseYaw(), rmseYaw() * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), " erro pos maximo: %.4f m", max_pos_err_);

    if (!results_file_.empty()) {
      std::ifstream probe(results_file_);
      const bool is_new = !probe.good() || probe.peek() == std::ifstream::traits_type::eof();
      probe.close();
      std::ofstream f(results_file_, std::ios::app);
      if (is_new) {
        f << "label,mode,kff,Omega,laps,rmse_pos_m,rmse_yaw_rad,max_pos_err_m\n";
      }
      f << label_ << ',' << mode_ << ',' << kff_ << ',' << W_ << ',' << laps_ << ','
        << rmsePos() << ',' << rmseYaw() << ',' << max_pos_err_ << '\n';
      RCLCPP_INFO(get_logger(), "Resultado acrescentado em %s", results_file_.c_str());
    }
    if (shutdown_when_done_) {
      rclcpp::shutdown();
    }
  }

  // parametros
  double A_, B_, W_, laps_, kff_, d_, v_max_, w_max_, align_tol_, start_delay_;
  std::string mode_, frame_id_, label_, results_file_;
  bool align_start_, shutdown_when_done_;
  control_toolbox::Pid pid_x_, pid_y_, pid_align_;

  // estado
  State state_{State::WAIT_ODOM};
  mrt::Pose2D pose_;
  bool have_odom_{false};
  double x0_{0.0}, y0_{0.0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time track_start_{0, 0, RCL_ROS_TIME};
  double sum_sq_pos_{0.0}, sum_sq_yaw_{0.0}, max_pos_err_{0.0};
  size_t n_samples_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rmse_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Figure8Tracker>());
  rclcpp::shutdown();
  return 0;
}
