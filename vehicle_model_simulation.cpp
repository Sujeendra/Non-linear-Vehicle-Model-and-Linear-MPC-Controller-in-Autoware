#include "simple_planning_simulator/vehicle_model/sim_model_nonlinear.hpp"

#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>

SimModelNonLinear::SimModelNonLinear(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor)
: SimModelInterface(6 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0))
{
  initializeInputQueue(dt);
  delta_time = dt;
}
double SimModelNonLinear::get_k()
{
  return k;
}
void SimModelNonLinear::set_k(double curvature)
{
  k = curvature;
}
double SimModelNonLinear::getX()
{
  return state_(IDX::X);
}
double SimModelNonLinear::getY()
{
  return state_(IDX::Y);
}
double SimModelNonLinear::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelNonLinear::getVx()
{
  return state_(IDX::VX);
}
double SimModelNonLinear::getVy()
{
  return 0.0;
}
double SimModelNonLinear::getAx()
{
  return state_(IDX::ACCX);
}
double SimModelNonLinear::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
double SimModelNonLinear::getSteer()
{
  // return measured values with bias added to actual values
  return state_(IDX::STEER) + steer_bias_;
}
void SimModelNonLinear::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);
  delta_time = dt;
  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  const auto prev_state = state_;
  updateRungeKutta(dt, delayed_input);

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // consider gear
  // update position and velocity first, and then acceleration is calculated naturally
  updateStateWithGear(state_, prev_state, gear_, dt);
}

void SimModelNonLinear::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}
void log_matrix(const std::string & name, const Eigen::MatrixXd & matrix)
{
  std::stringstream ss;
  ss << matrix;  // Converts the matrix to a string
  RCLCPP_ERROR(
    rclcpp::get_logger("printing matrices from non linear model"), "%s:\n%s", name.c_str(),
    ss.str().c_str());
}
Eigen::VectorXd SimModelNonLinear::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double acc = sat(state(IDX::ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  const double acc_des =
    sat(input(IDX_U::ACCX_DES), vx_rate_lim_, -vx_rate_lim_) * debug_acc_scaling_factor_;
  const double steer_des =
    sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;
  // NOTE: `steer_des` is calculated by control from measured values. getSteer() also gets the
  // measured value. The steer_rate used in the motion calculation is obtained from these
  // differences.
  const double steer_diff = getSteer() - steer_des;
  const double steer_diff_with_dead_band = std::invoke([&]() {
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  });
  const double steer_rate =
    sat(-steer_diff_with_dead_band / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);
  // find yaw from the nonlinear model
  double vx = std::max(vel, 0.01);

  double psi_des_dot = vx * get_k();
  const double mass_fl = 600;
  const double mass_fr = 600;
  const double mass_rl = 600;
  const double mass_rr = 600;
  const double cf = 155494.663;
  const double cr = 155494.663;
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  double mass = mass_front + mass_rear;
  double lf = (wheelbase_) * (1.0 - mass_front / mass);
  double lr = wheelbase_ * (1.0 - mass_rear / mass);
  double iz = lf * lf * mass_front + lr * lr * mass_rear;

  RCLCPP_ERROR(rclcpp::get_logger("printing details"), "lf %f", lf);
  RCLCPP_ERROR(rclcpp::get_logger("printing details"), "lr %f", lr);
  RCLCPP_ERROR(rclcpp::get_logger("printing details"), "mass %f", mass);
  RCLCPP_ERROR(rclcpp::get_logger("printing details"), "Iz %f", iz);

  double delta = steer;
  Eigen::MatrixXd a_d = Eigen::MatrixXd::Zero(4, 4);
  a_d(0, 1) = 1.0;
  a_d(1, 1) = -2 * (cf + cr) / (mass * vx);
  a_d(1, 2) = 2 * (cf + cr) / mass;
  a_d(1, 3) = 2 * (lr * cr - lf * cf) / (mass * vx);
  a_d(2, 3) = 1.0;
  a_d(3, 1) = 2 * (lr * cr - lf * cf) / (iz * vx);
  a_d(3, 2) = 2 * (lf * cf - lr * cr) / iz;
  a_d(3, 3) = -2 * (lf * lf * cf + lr * lr * cr) / (iz * vx);

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd a_d_inverse = (I - delta_time * 0.5 * a_d).inverse();

  a_d = a_d_inverse * (I + delta_time * 0.5 * a_d);  // bilinear discretization

  Eigen::MatrixXd b_d = Eigen::MatrixXd::Zero(4, 1);
  b_d(0, 0) = 0.0;
  b_d(1, 0) = 2 * cf / mass;
  b_d(2, 0) = 0.0;
  b_d(3, 0) = 2 * lf * cf / iz;

  Eigen::MatrixXd F_d = Eigen::MatrixXd::Zero(4, 2);
  F_d(1, 1) = 1 / mass;
  F_d(1, 0) = 1 / mass;
  F_d(3, 0) = lf / iz;
  F_d(3, 1) = -lr / iz;

  Eigen::MatrixXd f_d = Eigen::MatrixXd::Zero(2, 1);
  double alpha_f = steer - atan(lf * psi_des_dot / vx);
  double alpha_r = atan(lr * psi_des_dot / vx);
  double c2 = 40000;
  double c3 = 40000;
  int signf = 1;
  int signr = 1;
  if (alpha_f < 0) signf = -1;
  if (alpha_r < 0) signr = -1;
  f_d(0, 0) = -c2 * alpha_f * alpha_f * signf + c3 * alpha_f * alpha_f * alpha_f;
  f_d(1, 0) = -c2 * alpha_r * alpha_r * signr + c3 * alpha_r * alpha_r * alpha_r;

  Eigen::MatrixXd Ff_d = Eigen::MatrixXd::Zero(4, 1);
  Ff_d = F_d * f_d;

  Eigen::MatrixXd w_d = Eigen::MatrixXd::Zero(4, 1);
  w_d(0, 0) = 0.0;
  w_d(1, 0) = 2 * (lr * cr - lf * cf) / (mass * vx) - vx;
  w_d(2, 0) = 0.0;
  w_d(3, 0) = -2 * (lf * lf * cf + lr * lr * cr) / (iz * vx);
  // Log matrices

  b_d = (a_d_inverse * delta_time) * b_d;
  Ff_d = (a_d_inverse * delta_time) * Ff_d;
  w_d = (a_d_inverse * delta_time * psi_des_dot) * w_d;
  Eigen::MatrixXd Uex = Eigen::MatrixXd::Zero(1, 1);
  Uex(0, 0) = delta;
  // x0 should be zero very first time and then it needs to be Xex for the next iteration
  double yaw_error = 0;
  double lateral_error = 0;
  if (state(IDX::VX) > 0) {
    Eigen::VectorXd x0 = Prev_Xex;

    Eigen::VectorXd Xex = a_d * x0 + b_d * Uex + w_d * psi_des_dot + Ff_d;  // non linear model

    yaw_error = Prev_Xex(2);
    lateral_error = Prev_Xex(0);

    Prev_Xex = Xex;
  }

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = X - lateral_error * sin(yaw + yaw_error);
  d_state(IDX::Y) = Y + lateral_error * cos(yaw + yaw_error);
  d_state(IDX::YAW) = yaw_error + yaw;
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant_;

  return d_state;
}
void SimModelNonLinear::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  const auto setStopState = [&]() {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  };

  using autoware_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      setStopState();
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      setStopState();
    }
  } else {  // including 'gear == GearCommand::PARK'
    setStopState();
  }
}

