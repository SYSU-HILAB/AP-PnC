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
#include <std_msgs/msg/bool.hpp>
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_sim_solver_tailsitter_flu_sim.h"
#include "acados_solver_tailsitter_flu.h"

#include <spdlog/spdlog.h>
// #include "acados_solver_tailsitter_flu.h"
// #include "tailsitter_flu_cost/tailsitter_flu_cost.h"
// #include "tailsitter_flu_model/tailsitter_flu_model.h"
int status;  // acados operation state



// Template function to bind a member function to a std::function
template<typename T1, class T2>
std::function<void(const T1&)> bindHandler(void(T2::* func)(const T1&), T2* obj) {
  return std::bind(func, obj, std::placeholders::_1);
}




using StateMsg = interface::msg::State;
using ReferenceMsg = interface::msg::RefTrajectoryBullet;
using ControlMsg = interface::msg::Controls;
using TrajecotryPoint = interface::msg::TrajectoryPoint;

class AcadosInterface {
public:
  enum return_values
  {
    ACADOS_SUCCESS = 0,
    ACADOS_FAILURE,
    ACADOS_MAXITER,
    ACADOS_MINSTEP,
    ACADOS_QP_FAILURE,
    ACADOS_READY,
  };

private:
  std::function<int(int, void*)> set_reference_stage;
  std::function<int(int, double*)> set_model_parameter;
  std::function<int(void*)> set_current_state_lbx;
  std::function<int(void*)> set_current_state_ubx;
  std::function<void(void*)> get_nlp_out_u;
  tailsitter_flu_solver_capsule* acados_ocp_capsule =
    tailsitter_flu_acados_create_capsule();

  // tailsitter_flu
  tailsitter_flu_sim_sim_solver_capsule* sim_capsule =
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
    ocp_nlp_config* nlp_config =
      tailsitter_flu_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims* nlp_dims =
      tailsitter_flu_acados_get_nlp_dims(acados_ocp_capsule);
    // update N, nx, nu
    N = nlp_dims->N;
    nx = nlp_dims->nx[0];
    nu = nlp_dims->nu[0];

    ocp_nlp_in* nlp_in = tailsitter_flu_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out* nlp_out =
      tailsitter_flu_acados_get_nlp_out(acados_ocp_capsule);

    set_current_state_lbx =
      std::bind(ocp_nlp_constraints_model_set, nlp_config, nlp_dims, nlp_in,
        0, "lbx", std::placeholders::_1);
    set_current_state_ubx =
      std::bind(ocp_nlp_constraints_model_set, nlp_config, nlp_dims, nlp_in,
        0, "ubx", std::placeholders::_1);
    set_reference_stage =
      std::bind(ocp_nlp_cost_model_set, nlp_config, nlp_dims, nlp_in,
        std::placeholders::_1, "yref", std::placeholders::_2);
    set_model_parameter =
      std::bind(tailsitter_flu_acados_update_params, acados_ocp_capsule,
        std::placeholders::_1, std::placeholders::_2, 1);
    get_nlp_out_u = std::bind(ocp_nlp_out_get, nlp_config, nlp_dims, nlp_out, 0,
      "u", std::placeholders::_1);
    // ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
  };

public:
  int status;
  int N;
  int nx;
  int nu;
  static AcadosInterface& getInstance() {
    static AcadosInterface instance;
    return instance;
  }

  void setReference(double* y_ref, double* p, int i) {
    assert(i <= N);
    // the user should take care of y_ref is 16D
    set_reference_stage(i, y_ref);
    if (i < N)
      set_model_parameter(i, p);
  }

  void setReference(double* y_ref, int i) {
    assert(i == N);
    // the user should take care of y_ref is 12D
    set_reference_stage(i, y_ref);
  }

  void setState(double* x_current) {
    set_current_state_lbx(x_current);
    set_current_state_ubx(x_current);
  }

  int solve(void* u_sol) {
    status = tailsitter_flu_acados_solve(acados_ocp_capsule);
    get_nlp_out_u(u_sol);
    return status;
  }
};

class NmpcCallbacks {
public:
  virtual void stateCallback(const StateMsg& msg) = 0;
  virtual void referenceCallback(const ReferenceMsg& msg) = 0;
  virtual void initSubsPubs() = 0;
public:
  rclcpp::Subscription<StateMsg>::SharedPtr state_sub_;
  rclcpp::Subscription<ReferenceMsg>::SharedPtr ref_sub_;
  rclcpp::Publisher<ControlMsg>::SharedPtr ctrl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  const  rclcpp::QoS reliabe_qos_keep_last =
    rclcpp::QoS(1)
    .reliable()
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    .keep_last(1);

};



class NMPC : public rclcpp::Node, public NmpcCallbacks {
private:
  std::array<double, 4> u_sol_;
  std::array<double, 16> y_ref_;
  void initSubsPubs() override;
  void stateCallback(const StateMsg& msg) override;
  void referenceCallback(const ReferenceMsg& msg) override;

  void solve();

public:
  NMPC();
  ~NMPC();

private:
  std::vector<double> time_statistic;
  std::chrono::_V2::system_clock::time_point t_start_;
  std::chrono::_V2::system_clock::time_point t_end_;

  AcadosInterface& acados = AcadosInterface::getInstance();
  std::array<double, 4> u_sol;
};

void NMPC::initSubsPubs() {
  state_sub_ = this->create_subscription<StateMsg>(
    "/cur_state", reliabe_qos_keep_last,
    std::bind(&NMPC::stateCallback, this, std::placeholders::_1));

  ref_sub_ = this->create_subscription<ReferenceMsg>(
    "/ref_trajectory_bullet", reliabe_qos_keep_last,
    std::bind(&NMPC::referenceCallback, this, std::placeholders::_1));
  // Initialize publisher
  ctrl_pub_ = this->create_publisher<ControlMsg>("/mpc_controls", reliabe_qos_keep_last);

  ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ref_can_set", reliabe_qos_keep_last);
}

void NMPC::solve() {
  // calculate the solve function time
  // LOG 
  t_start_ = std::chrono::high_resolution_clock::now();
  status = acados.solve(u_sol.data());
  switch (status)
  {
  case AcadosInterface::return_values::ACADOS_SUCCESS:
  case AcadosInterface::return_values::ACADOS_MAXITER:
    break;
  case AcadosInterface::return_values::ACADOS_FAILURE:
    RCLCPP_FATAL(this->get_logger(), "acados_solve() returned status %d, it is ACADOS_FAILURE. Exiting.\n", status);
    exit(1);
    break;
  case AcadosInterface::return_values::ACADOS_MINSTEP:
    RCLCPP_FATAL(this->get_logger(), "acados_solve() returned status %d, it is ACADOS_MINSTEP. Exiting.\n", status);
    exit(1);
    break;
  case AcadosInterface::return_values::ACADOS_QP_FAILURE:
    RCLCPP_FATAL(this->get_logger(), "acados_solve() returned status %d, it is ACADOS_QP_FAILURE. Exiting.\n", status);
    exit(1);
    break;
  default:
    RCLCPP_FATAL(this->get_logger(), "acados_solve() returned status %d, it is ACADOS_READY.\n", status);
    break;
  }

  ControlMsg ctrl_msg;
  ctrl_msg.motors_u = { u_sol[0], u_sol[1], u_sol[2], u_sol[3] };
  ctrl_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  ctrl_pub_->publish(ctrl_msg);
  t_end_ = std::chrono::high_resolution_clock::now();
  time_statistic.emplace_back(
    std::chrono::duration<double, std::milli>(t_end_ - t_start_).count());
  spdlog::debug("control msg {}", ctrl_msg.motors_u[0]);
}


void NMPC::referenceCallback(const ReferenceMsg& msg) {
  // info entering the callback
  RCLCPP_INFO(this->get_logger(), "Entering referenceCallback");
  auto& refs = const_cast<ReferenceMsg&> (msg);
  int size = refs.points.size();
  RCLCPP_INFO(this->get_logger(), "size %d, N %d", size, acados.N);

  // TODO hahh
  assert(size >= acados.N + 1);
  // rclcpp print size and N
  for (int i = 0; i < acados.N; i++) {
    acados.setReference(refs.points[i].point.data(),
      &refs.points[i].cz_prediction, i);
  }
  acados.setReference(refs.points[acados.N].point.data(), acados.N);
  solve();

}

void NMPC::stateCallback(const StateMsg& msg) {
  msg.mpc_state.data();
  acados.setState(const_cast<double*>(msg.mpc_state.data()));
  //print state
  // printf("state: \n");
  // for (int i = 0; i < msg.mpc_state.size(); i++) {
  //   printf("%f ", msg.mpc_state[i]);
  // }
  using interface::msg::State;
  // init a quaterniond 
  Eigen::Quaterniond q(msg.mpc_state[State::QUAT_START_INDEX], msg.mpc_state[State::QUAT_START_INDEX + 1],
    msg.mpc_state[State::QUAT_START_INDEX + 2], msg.mpc_state[State::QUAT_START_INDEX + 3]);
  //print quaternion marked by w,x,y,z
  // printf("quaternion: %f %f %f %f\n", q.w(), q.x(), q.y(), q.z());
  //print rotation matrix
  // Eigen::Matrix3d rot = q.toRotationMatrix();
  // printf("rotation matrix: \n");
  // for (int i = 0; i < 3; i++) {
  //   for (int j = 0; j < 3; j++) {
  //     printf("%f ", rot(i, j));
  //   }
  //   printf("\n");
  // }
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = true;
  ready_pub_->publish(ready_msg);
  // double p = 0.0f;
  // // rclcpp info
  // RCLCPP_INFO(this->get_logger(), "stage nums %d", acados.N);
  // for (int i = 0; i < acados.N; i++) {
  //   acados.setReference(y_ref_.data(), &p, i);
  // }
  // // acados.setReference(y_ref.data(), &p, 0);

  // NMPC::solve();
  // // acados.solve(u_sol.data());
  // // // print u_sol
  // // printf("u_sol: \n");
  // for (int i = 0; i < 4; i++) {
  //   printf("%f ", u_sol[i]);
  // }
}

NMPC::NMPC() : Node("NMPC") {
  // Initialize the reference

  initSubsPubs();
  time_statistic.reserve(10000);
  // NMPC setup code here (not shown, specific to your application)
}


NMPC::~NMPC() {
  Eigen::VectorXd time_record;
  time_record.resize(time_statistic.size());
  for (int i = 0; i < time_statistic.size(); i++) {
    time_record[i] = time_statistic[i];
  }
  spdlog::info("average estimation time {} ms", time_record.mean());
  spdlog::info("max estimation time {} ms", time_record.maxCoeff());
  spdlog::info("min estimation time {} ms", time_record.minCoeff());
}


int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::debug);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NMPC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
