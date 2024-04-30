// #include "tailsitter_flu_app.h"
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>
#include <chrono>
#include <interface/msg/controls.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_sim_solver_tailsitter_flu_sim.h"
#include "acados_solver_tailsitter_flu.h"
// #include "acados_solver_tailsitter_flu.h"
// #include "tailsitter_flu_cost/tailsitter_flu_cost.h"
// #include "tailsitter_flu_model/tailsitter_flu_model.h"
int status;  // acados operation state

// int idxbx0[3];

// double min_time = 1e12;
// double elapsed_time;

// double x_target[3];
// double x_state[5];
// double x_current[3];
// double u_current[3];
// int N;
// int nx;
// int nu;

using StateMsg = interface::msg::State;
using ReferenceMsg = interface::msg::RefTrajectoryBullet;
using ControlMsg = interface::msg::Controls;
using TrajecotryPoint = interface::msg::TrajectoryPoint;

class AcadosInterface {
 private:
  std::function<int(void *, int)> set_reference_stage;
  std::function<int(double *, int)> set_model_parameter;
  std::function<int(void *)> set_current_state_lbx;
  std::function<int(void *)> set_current_state_ubx;
  std::function<void(void *)> get_nlp_out_u;
  tailsitter_flu_solver_capsule *acados_ocp_capsule =
      tailsitter_flu_acados_create_capsule();

  // tailsitter_flu
  tailsitter_flu_sim_sim_solver_capsule *sim_capsule =
      tailsitter_flu_sim_acados_sim_solver_create_capsule();

  AcadosInterface() {
    status = tailsitter_flu_acados_create(acados_ocp_capsule);
    if (status) {
      printf("acados_create() Ocp returned status %d. Exiting.\n", status);
      exit(1);
    }
    status = tailsitter_flu_sim_acados_sim_create(sim_capsule);
    if (status) {
      printf("acados_create() simulator returned status %d. Exiting.\n",
             status);
      exit(1);
    }
    auto sim_config = tailsitter_flu_sim_acados_get_sim_config(sim_capsule);
    auto sim_dims = tailsitter_flu_sim_acados_get_sim_dims(sim_capsule);
    auto sim_in = tailsitter_flu_sim_acados_get_sim_in(sim_capsule);
    auto sim_out = tailsitter_flu_sim_acados_get_sim_out(sim_capsule);

    // some important structure of ocp
    ocp_nlp_config *nlp_config =
        tailsitter_flu_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims =
        tailsitter_flu_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = tailsitter_flu_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out =
        tailsitter_flu_acados_get_nlp_out(acados_ocp_capsule);

    set_current_state_lbx =
        std::bind(ocp_nlp_constraints_model_set, nlp_config, nlp_dims, nlp_in,
                  0, "lbx", std::placeholders::_1);
    set_current_state_ubx =
        std::bind(ocp_nlp_constraints_model_set, nlp_config, nlp_dims, nlp_in,
                  0, "ubx", std::placeholders::_1);
    set_reference_stage =
        std::bind(ocp_nlp_cost_model_set, nlp_config, nlp_dims, nlp_in,
                  std::placeholders::_2, "yref", std::placeholders::_1);
    set_model_parameter =
        std::bind(ocp_nlp_cost_model_set, nlp_config, nlp_dims, nlp_in,
                  std::placeholders::_2, "p", std::placeholders::_1);
    get_nlp_out_u = std::bind(ocp_nlp_out_get, nlp_config, nlp_dims, nlp_out, 0,
                              "u", std::placeholders::_1);
    // ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
  };

 public:
  int status;
  int N;
  int nx;
  int nu;
  static AcadosInterface &getInstance() {
    static AcadosInterface instance;
    return instance;
  }

  void setReference(double *y_ref, double *p, int i) {
    set_reference_stage(y_ref, i);
    set_model_parameter(p, i);
  }

  void setState(double *x_current) {
    set_current_state_lbx(x_current);
    set_current_state_ubx(x_current);
  }

  int solve(void *u_sol) {
    status = tailsitter_flu_acados_solve(acados_ocp_capsule);
    get_nlp_out_u(u_sol);
    return status;
  }
};
class NMPC : public rclcpp::Node {
 public:
  NMPC() : Node("NMPC") {
    // Initialize subscriber
    state_sub_ = this->create_subscription<StateMsg>(
        "/cur_state", 10,
        std::bind(&NMPC::state_cb, this, std::placeholders::_1));

    ref_sub_ = this->create_subscription<ReferenceMsg>(
        "/ref_bullet", 10,
        std::bind(&NMPC::ref_cb, this, std::placeholders::_1));
    // Initialize publisher
    ctrl_pub_ = this->create_publisher<ControlMsg>("/mpc_ctrls", 10);
    optimization_thread_ = std::thread(&NMPC::solve, this);
    // NMPC setup code here (not shown, specific to your application)
  }
  ~NMPC() {
    optimization_thread_.join();
    Eigen::VectorXd time_record;
    time_record.resize(time_statistic.size());
    for (int i = 0; i < time_statistic.size(); i++) {
      time_record[i] = time_statistic[i];
    }
    printf("average estimation time %f ms \n", time_record.mean());
    printf("max estimation time %f ms \n", time_record.maxCoeff());
    printf("min estimation time %f ms \n", time_record.minCoeff());
  }

 private:
  std::vector<double> time_statistic;
  void state_cb(const StateMsg::SharedPtr msg) {
    acados.setState(msg->mpc_state.data());
  }

  void ref_cb(const ReferenceMsg::SharedPtr msg) {
    auto &refs = *msg;
    int size = refs.points.size();
    assert(size == acados.N + 1);

    for (int i = 0; i < size; i++) {
      acados.setReference(refs.points[i].point.data(),
                          &refs.points[i].cz_prediction, i);
    }
    optimization_thread_ = std::thread(&NMPC::solve, this);
    optimization_thread_.detach();
  }

  void solve() {
    // calculate the solve function time
    auto t_start = std::chrono::high_resolution_clock::now();
    status = acados.solve(u_sol.data());
    if (status) {
      printf("acados_solve() returned status %d. Exiting.\n", status);
      exit(1);
    }
    ControlMsg ctrl_msg;
    ctrl_msg.motors_u = {u_sol[0], u_sol[1], u_sol[2], u_sol[3]};
    ctrl_pub_->publish(ctrl_msg);
    auto t_end = std::chrono::high_resolution_clock::now();
    time_statistic.emplace_back(
        std::chrono::duration<double, std::milli>(t_end - t_start).count());
  }

  rclcpp::Subscription<StateMsg>::SharedPtr state_sub_;
  rclcpp::Subscription<ReferenceMsg>::SharedPtr ref_sub_;
  rclcpp::Publisher<ControlMsg>::SharedPtr ctrl_pub_;

  AcadosInterface &acados = AcadosInterface::getInstance();
  std::thread optimization_thread_;
  std::array<double, 4> u_sol;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NMPC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
