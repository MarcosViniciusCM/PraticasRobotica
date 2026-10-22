// Pratica 04 - Exercicio 2.1: Seguimento da trajetoria em 8 com sensores (GPS e IMU).
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "my_robot_sensors/common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mrt = my_robot_sensors;

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

    use_sensors_ = declare_parameter("use_sensors", true);
    gps_topic_ = declare_parameter("gps_topic", std::string("gps/pose"));
    imu_topic_ = declare_parameter("imu_topic", std::string("imu/data"));
    // O yaw da IMU nao esta no mesmo referencial do GPS (offset medido ~165 graus).
    // calibrate_yaw: o robo anda em linha reta e o offset sai do rumo medido pelo GPS.
    calibrate_yaw_ = declare_parameter("calibrate_yaw", true);
    yaw_offset_ = declare_parameter("imu_yaw_offset", 0.0);
    cal_speed_ = declare_parameter("cal_speed", 0.3);
    cal_distance_ = declare_parameter("cal_distance", 1.5);
    cal_rest_time_ = declare_parameter("cal_rest_time", 1.5);

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
    sensor_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "figure8/sensor_pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "figure8/desired_path", rclcpp::QoS(1).transient_local());
    error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("figure8/error", 10);
    real_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("figure8/real_error", 10);
    rmse_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("figure8/rmse", 10);

    // Odometria de referencia (ground truth)
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        gt_pose_.x = m->pose.pose.position.x;
        gt_pose_.y = m->pose.pose.position.y;
        gt_pose_.yaw = mrt::yawFromQuaternion(m->pose.pose.orientation);
        have_gt_ = true;

        if (!use_sensors_) {
          pose_ = gt_pose_;
          have_pose_ = true;
          step(rclcpp::Time(m->header.stamp, RCL_ROS_TIME));
        }
      });

    if (use_sensors_) {
      gps_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        gps_topic_, 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          last_gps_x_ = m->pose.position.x;
          last_gps_y_ = m->pose.position.y;
          have_gps_ = true;
        });

      // Disparo do controle a 50 Hz acoplado a leitura da IMU
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, [this](sensor_msgs::msg::Imu::SharedPtr m) {
          if (!have_gps_) {
            return;
          }
          pose_.x = last_gps_x_;
          pose_.y = last_gps_y_;
          imu_yaw_raw_ = mrt::yawFromQuaternion(m->orientation);
          pose_.yaw = mrt::wrapAngle(imu_yaw_raw_ + yaw_offset_);
          have_pose_ = true;

          geometry_msgs::msg::PoseStamped sp;
          sp.header.stamp = m->header.stamp;
          sp.header.frame_id = frame_id_;
          sp.pose.position.x = pose_.x;
          sp.pose.position.y = pose_.y;
          sp.pose.orientation = m->orientation;
          sensor_pose_pub_->publish(sp);

          step(rclcpp::Time(m->header.stamp, RCL_ROS_TIME));
        });
    }

    RCLCPP_INFO(get_logger(),
      "Trajetoria em 8: A=%.2f B=%.2f Omega=%.3f rad/s (T=%.2f s), %.1f voltas, modo=%s, "
      "Kff=%.2f, d=%.2f, use_sensors=%s",
      A_, B_, W_, 2.0 * M_PI / W_, laps_, mode_.c_str(), kff_, d_,
      use_sensors_ ? "true" : "false");

    const double v0 = std::hypot(A_ * W_, B_ * W_);
    if (v0 > v_max_) {
      RCLCPP_WARN(get_logger(),
        "A velocidade de pico da referencia (%.2f m/s) excede max_linear_velocity (%.2f)",
        v0, v_max_);
    }
  }

private:
  enum class State {
    WAIT_ODOM, CAL_REST, CAL_DRIVE, CAL_SETTLE, WAIT_START, ALIGN, TRACK, DONE
  };

  struct Ref
  {
    double x, y, dx, dy, ddx, ddy;
  };

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
        if (have_pose_) {
          if (use_sensors_ && calibrate_yaw_) {
            state_ = State::CAL_REST;
            phase_start_ = now_t;
            resetAcc();
            RCLCPP_INFO(get_logger(), "Calibrando yaw da IMU (%.1f m em linha reta)",
              cal_distance_);
            return;
          }
          x0_ = pose_.x;
          y0_ = pose_.y;
          publishPath();
          state_ = State::WAIT_START;
          phase_start_ = now_t;
          RCLCPP_INFO(get_logger(), "Origem da trajetoria definida em (%.2f, %.2f)",
            x0_, y0_);
        }
        return;

      case State::CAL_REST:
        cmd_pub_->publish(mrt::makeTwist(0.0, 0.0));
        accumulate();
        if ((now_t - phase_start_).seconds() >= cal_rest_time_) {
          p0x_ = acc_x_ / acc_n_;
          p0y_ = acc_y_ / acc_n_;
          sum_sin_ = sum_cos_ = 0.0;
          state_ = State::CAL_DRIVE;
          phase_start_ = now_t;
        }
        return;

      case State::CAL_DRIVE:
        cmd_pub_->publish(mrt::makeTwist(cal_speed_, 0.0));
        sum_sin_ += std::sin(imu_yaw_raw_);
        sum_cos_ += std::cos(imu_yaw_raw_);
        if ((now_t - phase_start_).seconds() >= cal_distance_ / cal_speed_) {
          state_ = State::CAL_SETTLE;
          phase_start_ = now_t;
          resetAcc();
        }
        return;

      case State::CAL_SETTLE: {
          cmd_pub_->publish(mrt::makeTwist(0.0, 0.0));
          // so acumula depois que o robo parou (descarta o 1o segundo)
          if ((now_t - phase_start_).seconds() >= 1.0) {
            accumulate();
          }
          if ((now_t - phase_start_).seconds() >= 1.0 + cal_rest_time_ && acc_n_ > 0) {
            const double p1x = acc_x_ / acc_n_;
            const double p1y = acc_y_ / acc_n_;
            const double dist = std::hypot(p1x - p0x_, p1y - p0y_);
            const double heading = std::atan2(p1y - p0y_, p1x - p0x_);
            const double mean_yaw = std::atan2(sum_sin_, sum_cos_);
            yaw_offset_ = mrt::wrapAngle(heading - mean_yaw);
            RCLCPP_INFO(get_logger(),
              "Calibracao: deslocamento %.2f m, rumo GPS %.1f deg, yaw IMU %.1f deg -> "
              "offset %.1f deg", dist, heading * 180.0 / M_PI, mean_yaw * 180.0 / M_PI,
              yaw_offset_ * 180.0 / M_PI);
            if (dist < 0.5 * cal_distance_) {
              RCLCPP_WARN(get_logger(), "Deslocamento menor que o esperado: offset incerto");
            }
            pose_.yaw = mrt::wrapAngle(imu_yaw_raw_ + yaw_offset_);
            x0_ = p1x;
            y0_ = p1y;
            publishPath();
            state_ = State::WAIT_START;
            phase_start_ = now_t;
            RCLCPP_INFO(get_logger(), "Origem da trajetoria definida em (%.2f, %.2f)",
              x0_, y0_);
          }
          return;
        }

      case State::WAIT_START:
        if ((now_t - phase_start_).seconds() >= start_delay_) {
          state_ = align_start_ ? State::ALIGN : State::TRACK;
          phase_start_ = now_t;
          if (align_start_) {
            RCLCPP_INFO(get_logger(), "Alinhando com a orientacao inicial da trajetoria");
          } else {
            startTracking(now_t);
          }
        }
        return;

      case State::ALIGN: {
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

  void resetAcc() {acc_x_ = acc_y_ = 0.0; acc_n_ = 0;}
  void accumulate() {acc_x_ += pose_.x; acc_y_ += pose_.y; ++acc_n_;}

  void startTracking(const rclcpp::Time & t)
  {
    // A odometria das rodas parte de outro referencial (origem e yaw proprios,
    // e ela acumula deriva por escorregamento). Alinha-se a ela ao pose atual.
    gt_ref_x_ = gt_pose_.x;
    gt_ref_y_ = gt_pose_.y;
    gt_dpsi_ = mrt::wrapAngle(pose_.yaw - gt_pose_.yaw);
    pose_ref_x_ = pose_.x;
    pose_ref_y_ = pose_.y;
    state_ = State::TRACK;
    track_start_ = t;
    pid_x_.reset();
    pid_y_.reset();
    sum_sq_pos_ = sum_sq_yaw_ = max_pos_err_ = 0.0;
    sum_sq_gt_pos_ = sum_sq_gt_yaw_ = max_gt_pos_err_ = 0.0;
    n_samples_ = 0;
    RCLCPP_INFO(get_logger(), "Iniciando seguimento (modo=%s, use_sensors=%s)",
      mode_.c_str(), use_sensors_ ? "true" : "false");
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
      const double px = pose_.x + d_ * std::cos(th);
      const double py = pose_.y + d_ * std::sin(th);
      const double pdx = r.x + d_ * std::cos(th_d);
      const double pdy = r.y + d_ * std::sin(th_d);
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

    // Erro sob a otica do controle (alimentado por sensores ou odom)
    const double ex = r.x - pose_.x;
    const double ey = r.y - pose_.y;
    const double eth = mrt::wrapAngle(th_d - pose_.yaw);
    const double epos = std::hypot(ex, ey);
    sum_sq_pos_ += epos * epos;
    sum_sq_yaw_ += eth * eth;
    max_pos_err_ = std::max(max_pos_err_, epos);

    // Erro real em relacao ao ground truth (/odom)
    if (have_gt_) {
      const double rx = gt_pose_.x - gt_ref_x_;
      const double ry = gt_pose_.y - gt_ref_y_;
      const double gx = pose_ref_x_ + std::cos(gt_dpsi_) * rx - std::sin(gt_dpsi_) * ry;
      const double gy = pose_ref_y_ + std::sin(gt_dpsi_) * rx + std::cos(gt_dpsi_) * ry;
      const double gtex = r.x - gx;
      const double gtey = r.y - gy;
      const double gteth = mrt::wrapAngle(th_d - mrt::wrapAngle(gt_pose_.yaw + gt_dpsi_));
      const double gtepos = std::hypot(gtex, gtey);
      sum_sq_gt_pos_ += gtepos * gtepos;
      sum_sq_gt_yaw_ += gteth * gteth;
      max_gt_pos_err_ = std::max(max_gt_pos_err_, gtepos);

      geometry_msgs::msg::Vector3Stamped real_err;
      real_err.header.stamp = now_t;
      real_err.header.frame_id = frame_id_;
      real_err.vector.x = gtex;
      real_err.vector.y = gtey;
      real_err.vector.z = gteth;
      real_error_pub_->publish(real_err);
    }
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
  double rmseGtPos() const {return n_samples_ ? std::sqrt(sum_sq_gt_pos_ / n_samples_) : 0.0;}
  double rmseGtYaw() const {return n_samples_ ? std::sqrt(sum_sq_gt_yaw_ / n_samples_) : 0.0;}

  void finish()
  {
    state_ = State::DONE;
    cmd_pub_->publish(mrt::makeTwist(0.0, 0.0));
    RCLCPP_INFO(get_logger(), "==================== TRAJETORIA CONCLUIDA %s", label_.c_str());
    RCLCPP_INFO(get_logger(), " modo=%s Kff=%.2f Omega=%.3f voltas=%.1f use_sensors=%s",
      mode_.c_str(), kff_, W_, laps_, use_sensors_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), " [Controle]     RMSE pos: %.4f m | RMSE yaw: %.4f rad | Max erro: %.4f m",
      rmsePos(), rmseYaw(), max_pos_err_);
    if (have_gt_) {
      RCLCPP_INFO(get_logger(), " [Ground Truth] RMSE pos: %.4f m | RMSE yaw: %.4f rad | Max erro: %.4f m",
        rmseGtPos(), rmseGtYaw(), max_gt_pos_err_);
    }

    if (!results_file_.empty()) {
      std::ifstream probe(results_file_);
      const bool is_new = !probe.good() || probe.peek() == std::ifstream::traits_type::eof();
      probe.close();
      std::ofstream f(results_file_, std::ios::app);
      if (is_new) {
        f << "label,mode,kff,Omega,laps,use_sensors,rmse_pos_m,rmse_yaw_rad,max_pos_err_m,gt_rmse_pos_m,gt_rmse_yaw_rad,gt_max_pos_err_m\n";
      }
      f << label_ << ',' << mode_ << ',' << kff_ << ',' << W_ << ',' << laps_ << ','
        << (use_sensors_ ? "true" : "false") << ','
        << rmsePos() << ',' << rmseYaw() << ',' << max_pos_err_ << ','
        << rmseGtPos() << ',' << rmseGtYaw() << ',' << max_gt_pos_err_ << '\n';
      RCLCPP_INFO(get_logger(), "Resultado acrescentado em %s", results_file_.c_str());
    }
    if (shutdown_when_done_) {
      rclcpp::shutdown();
    }
  }

  double A_, B_, W_, laps_, kff_, d_, v_max_, w_max_, align_tol_, start_delay_;
  std::string mode_, frame_id_, label_, results_file_;
  bool align_start_, shutdown_when_done_, use_sensors_;
  std::string gps_topic_, imu_topic_;
  control_toolbox::Pid pid_x_, pid_y_, pid_align_;

  bool calibrate_yaw_;
  double yaw_offset_, cal_speed_, cal_distance_, cal_rest_time_;
  double imu_yaw_raw_{0.0};
  double acc_x_{0.0}, acc_y_{0.0}, p0x_{0.0}, p0y_{0.0}, sum_sin_{0.0}, sum_cos_{0.0};
  int acc_n_{0};
  double gt_ref_x_{0.0}, gt_ref_y_{0.0}, gt_dpsi_{0.0}, pose_ref_x_{0.0}, pose_ref_y_{0.0};

  State state_{State::WAIT_ODOM};
  mrt::Pose2D pose_;
  mrt::Pose2D gt_pose_;
  bool have_pose_{false};
  bool have_gt_{false};
  bool have_gps_{false};
  double last_gps_x_{0.0};
  double last_gps_y_{0.0};
  double x0_{0.0}, y0_{0.0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time track_start_{0, 0, RCL_ROS_TIME};
  double sum_sq_pos_{0.0}, sum_sq_yaw_{0.0}, max_pos_err_{0.0};
  double sum_sq_gt_pos_{0.0}, sum_sq_gt_yaw_{0.0}, max_gt_pos_err_{0.0};
  size_t n_samples_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr real_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rmse_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Figure8Tracker>());
  rclcpp::shutdown();
  return 0;
}
