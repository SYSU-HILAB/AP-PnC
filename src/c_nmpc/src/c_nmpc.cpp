// #include "tailsitter_flu_app.h"
#include <spdlog/spdlog.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <interface/msg/controls.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/tracking_info.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_tailsitter_flu.h"
// #include "acados_solver_tailsitter_flu.h"
// #include "tailsitter_flu_cost/tailsitter_flu_cost.h"
// #include "tailsitter_flu_model/tailsitter_flu_model.h"
int status;  // acados operation state

// Template function to bind a member function to a std::function
template <typename T1, class T2>
std::function<void(const T1&)> bindHandler(void (T2::*func)(const T1&), T2* obj)
{
    return std::bind(func, obj, std::placeholders::_1);
}

using StateMsg        = interface::msg::State;
using ReferenceBullet = interface::msg::RefTrajectoryBullet;
using ControlMsg      = interface::msg::Controls;
using TrajecotryPoint = interface::msg::TrajectoryPoint;
using TrackingInfoMsg = interface::msg::TrackingInfo;
class AcadosInterface
{
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
    std::function<int(int, void*)>   set_reference_stage;
    std::function<int(int, double*)> set_model_parameter;
    std::function<int(void*)>        set_current_state_lbx;
    std::function<int(void*)>        set_current_state_ubx;
    std::function<void(void*)>       get_nlp_out_u;
    std::function<void(void*)>       get_nlp_out_x_next;
    tailsitter_flu_solver_capsule*   acados_ocp_capsule =
        tailsitter_flu_acados_create_capsule();

    AcadosInterface()
    {
        status = tailsitter_flu_acados_create(acados_ocp_capsule);
        if (status)
        {
            printf("acados_create() Ocp returned status %d. Exiting.\n",
                   status);
            exit(1);
        }
        if (status)
        {
            printf("acados_create() simulator returned status %d. Exiting.\n",
                   status);
            exit(1);
        }

        // some important structure of ocp
        ocp_nlp_config* nlp_config =
            tailsitter_flu_acados_get_nlp_config(acados_ocp_capsule);
        ocp_nlp_dims* nlp_dims =
            tailsitter_flu_acados_get_nlp_dims(acados_ocp_capsule);
        // update N, nx, nu
        N  = nlp_dims->N;
        nx = nlp_dims->nx[0];
        nu = nlp_dims->nu[0];

        ocp_nlp_in* nlp_in =
            tailsitter_flu_acados_get_nlp_in(acados_ocp_capsule);
        ocp_nlp_out* nlp_out =
            tailsitter_flu_acados_get_nlp_out(acados_ocp_capsule);

        set_current_state_lbx =
            std::bind(ocp_nlp_constraints_model_set, nlp_config, nlp_dims,
                      nlp_in, 0, "lbx", std::placeholders::_1);
        set_current_state_ubx =
            std::bind(ocp_nlp_constraints_model_set, nlp_config, nlp_dims,
                      nlp_in, 0, "ubx", std::placeholders::_1);
        set_reference_stage =
            std::bind(ocp_nlp_cost_model_set, nlp_config, nlp_dims, nlp_in,
                      std::placeholders::_1, "yref", std::placeholders::_2);
        set_model_parameter =
            std::bind(tailsitter_flu_acados_update_params, acados_ocp_capsule,
                      std::placeholders::_1, std::placeholders::_2, 1);
        get_nlp_out_u      = std::bind(ocp_nlp_out_get, nlp_config, nlp_dims,
                                       nlp_out, 0, "u", std::placeholders::_1);
        get_nlp_out_x_next = std::bind(ocp_nlp_out_get, nlp_config, nlp_dims,
                                       nlp_out, 1, "x", std::placeholders::_1);
        // ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
    };

   public:
    int                     status;
    int                     N;
    int                     nx;
    int                     nu;
    static AcadosInterface& getInstance()
    {
        static AcadosInterface instance;
        return instance;
    }

    void setReference(double* y_ref, double* p, int i)
    {
        assert(i <= N);
        // the user should take care of y_ref is 16D
        set_reference_stage(i, y_ref);
        if (i < N)
            set_model_parameter(i, p);
    }

    void setReference(double* y_ref, int i)
    {
        assert(i == N);
        // the user should take care of y_ref is 12D
        set_reference_stage(i, y_ref);
    }

    void setState(double* x_current)
    {
        set_current_state_lbx(x_current);
        set_current_state_ubx(x_current);
    }

    // solve result is u_sol & x_next
    int solve(void* u_sol, void* x_next)
    {
        status = tailsitter_flu_acados_solve(acados_ocp_capsule);
        get_nlp_out_u(u_sol);
        get_nlp_out_x_next(x_next);
        return status;
    }
};

class NmpcCallbacks
{
   public:
    virtual void stateCallback(const StateMsg& msg)            = 0;
    virtual void referenceCallback(const ReferenceBullet& msg) = 0;
    virtual void initSubsPubs()                                = 0;

   public:
    rclcpp::Subscription<StateMsg>::SharedPtr         state_sub_;
    rclcpp::Subscription<ReferenceBullet>::SharedPtr  ref_sub_;
    rclcpp::Publisher<ControlMsg>::SharedPtr          ctrl_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::Publisher<TrackingInfoMsg>::SharedPtr     tracking_info_pub_;
    const rclcpp::QoS                                 reliabe_qos_keep_last =
        rclcpp::QoS(10)
            .reliable()
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            .keep_last(1);
};

class NMPC : public rclcpp::Node, public NmpcCallbacks
{
   private:
    // std::array<double, 4> u_sol_;
    // std::array<double, 16> y_ref_;
    void          initSubsPubs() override;
    void          stateCallback(const StateMsg& msg) override;
    void          referenceCallback(const ReferenceBullet& msg) override;
    double        mass_;
    double        cz_predict_;
    std::ofstream file;

    void solve();

   public:
    NMPC();
    ~NMPC();

   private:
    std::vector<double>                        time_statistic;
    std::chrono::_V2::system_clock::time_point t_start_;
    std::chrono::_V2::system_clock::time_point t_end_;
    AcadosInterface& acados = AcadosInterface::getInstance();

    constexpr static int controls_size    = 4;
    constexpr static int state_size       = 13;
    constexpr static int pos_start_index  = 0;
    constexpr static int vel_start_index  = 3;
    constexpr static int rate_start_index = 6;
    constexpr static int q_start_index    = 9;

    std::array<double, state_size + controls_size> x_cur_;
    std::array<double, controls_size>              u_;
    // u_sol is now du
    std::array<double, controls_size> u_sol;
    // p,v,r,q
    // augment the state with the controls
    std::array<double, state_size + controls_size> x_next;
    // store reference msg
};

void NMPC::initSubsPubs()
{
    state_sub_ = this->create_subscription<StateMsg>(
        "/cur_state", reliabe_qos_keep_last,
        std::bind(&NMPC::stateCallback, this, std::placeholders::_1));

    ref_sub_ = this->create_subscription<ReferenceBullet>(
        "/ref_trajectory_bullet", reliabe_qos_keep_last,
        std::bind(&NMPC::referenceCallback, this, std::placeholders::_1));
    // Initialize publisher
    ctrl_pub_ = this->create_publisher<ControlMsg>("/mpc_controls",
                                                   reliabe_qos_keep_last);

    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/ref_can_set", reliabe_qos_keep_last);

    tracking_info_pub_ = this->create_publisher<TrackingInfoMsg>(
        "/tracking_info", reliabe_qos_keep_last);
}

void NMPC::solve()
{
    // calculate the solve function time
    // LOG
    status = acados.solve(u_sol.data(), x_next.data());

    switch (status)
    {
        case AcadosInterface::return_values::ACADOS_SUCCESS:
        case AcadosInterface::return_values::ACADOS_MAXITER:
            break;
        case AcadosInterface::return_values::ACADOS_FAILURE:
            RCLCPP_FATAL(this->get_logger(),
                         "acados_solve() returned status %d, it is "
                         "ACADOS_FAILURE. Exiting.\n",
                         status);
            exit(1);
            break;
        case AcadosInterface::return_values::ACADOS_MINSTEP:
            RCLCPP_FATAL(this->get_logger(),
                         "acados_solve() returned status %d, it is "
                         "ACADOS_MINSTEP. Exiting.\n",
                         status);
            exit(1);
            break;
        case AcadosInterface::return_values::ACADOS_QP_FAILURE:
            RCLCPP_FATAL(this->get_logger(),
                         "acados_solve() returned status %d, it is "
                         "ACADOS_QP_FAILURE. Exiting.\n",
                         status);
            exit(1);
            break;
        default:
            RCLCPP_FATAL(
                this->get_logger(),
                "acados_solve() returned status %d, it is ACADOS_READY.\n",
                status);
            break;
    }

    Eigen::Quaterniond q_next(x_next[q_start_index], x_next[q_start_index + 1],
                              x_next[q_start_index + 2],
                              x_next[q_start_index + 3]);
    q_next.normalize();

    double pitch = q_next.toRotationMatrix().eulerAngles(2, 1, 0)(1);
    // wrap pitch to [-pi,pi]
    if (pitch >= M_PI_2){
        pitch -= M_PI;
    }
    if (pitch < -M_PI_2){
        pitch += M_PI;
    }
    // copy x_aug controls to u_
    std::copy(x_next.data() + state_size,
              x_next.data() + state_size + controls_size, u_.begin());
    ControlMsg ctrl_msg;
    ctrl_msg.motors_u = {u_[0], u_[1], u_[2], u_[3]};
    ctrl_msg.rates_sp = {x_next[rate_start_index], x_next[rate_start_index + 1],
                         x_next[rate_start_index + 2]};
    ctrl_msg.timestamp     = this->get_clock()->now().nanoseconds() / 1000;
    ctrl_msg.pitch_sp      = pitch;
    ctrl_msg.thrust_acc_sp = (u_[0] + u_[1] + u_[2] + u_[3]) / mass_;
    ctrl_pub_->publish(ctrl_msg);

    t_end_ = std::chrono::high_resolution_clock::now();
    time_statistic.emplace_back(
        std::chrono::duration<double, std::milli>(t_end_ - t_start_).count());
}

void NMPC::referenceCallback(const ReferenceBullet& msg)
{
    // info entering the callback
    auto& refs           = const_cast<ReferenceBullet&>(msg);
    bool  tracking_valid = refs.tracking_valid;
    cz_predict_          = refs.points[0].cz_prediction;
    TrackingInfoMsg tracking_info_msg;
    tracking_info_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    tracking_info_msg.ref_position = {
        static_cast<float>(
            refs.points[0].point[TrajecotryPoint::POS_START_INDEX]),
        static_cast<float>(
            refs.points[0].point[TrajecotryPoint::POS_START_INDEX + 1]),
        static_cast<float>(
            refs.points[0].point[TrajecotryPoint::POS_START_INDEX + 2])};
    tracking_info_msg.ref_velocity = {
        static_cast<float>(
            refs.points[0].point[TrajecotryPoint::VEL_START_INDEX]),
        static_cast<float>(
            refs.points[0].point[TrajecotryPoint::VEL_START_INDEX + 1]),
        static_cast<float>(
            refs.points[0].point[TrajecotryPoint::VEL_START_INDEX + 2])};
    tracking_info_msg.actual_position = {
        static_cast<float>(x_cur_[pos_start_index]),
        static_cast<float>(x_cur_[pos_start_index + 1]),
        static_cast<float>(x_cur_[pos_start_index + 2])};
    tracking_info_msg.actual_velocity = {
        static_cast<float>(x_cur_[vel_start_index]),
        static_cast<float>(x_cur_[vel_start_index + 1]),
        static_cast<float>(x_cur_[vel_start_index + 2])};
    tracking_info_msg.actual_vxy = tracking_info_msg.actual_velocity[0] *
                                       tracking_info_msg.actual_velocity[0] +
                                   tracking_info_msg.actual_velocity[1] *
                                       tracking_info_msg.actual_velocity[1];
    tracking_info_msg.ref_vxy =
        tracking_info_msg.ref_velocity[0] * tracking_info_msg.ref_velocity[0] +
        tracking_info_msg.ref_velocity[1] * tracking_info_msg.ref_velocity[1];
    tracking_info_msg.ref_yb = {
        static_cast<float>(
            msg.points[0].point[TrajecotryPoint::YB_START_INDEX]),
        static_cast<float>(
            msg.points[0].point[TrajecotryPoint::YB_START_INDEX + 1]),
        static_cast<float>(
            msg.points[0].point[TrajecotryPoint::YB_START_INDEX + 2])};
    Eigen::Quaterniond q_cur(x_cur_[q_start_index], x_cur_[q_start_index + 1],
                             x_cur_[q_start_index + 2],
                             x_cur_[q_start_index + 3]);
    q_cur.normalize();
    auto yb_double              = q_cur.toRotationMatrix().col(1);
    tracking_info_msg.actual_yb = {static_cast<float>(yb_double[0]),
                                   static_cast<float>(yb_double[1]),
                                   static_cast<float>(yb_double[2])};

    Eigen::Vector3d vb =
        q_cur.conjugate().toRotationMatrix() *
        Eigen::Vector3d(x_cur_[vel_start_index], x_cur_[vel_start_index + 1],
                        x_cur_[vel_start_index + 2]);

    tracking_info_msg.alpha = static_cast<float>(atan2(vb(0), vb(2)));
    tracking_info_msg.cz    = static_cast<float>(cz_predict_);
    // tracking_info_msg.cz =
    if (tracking_valid)
        tracking_info_pub_->publish(tracking_info_msg);

    // Debug
    int size = refs.points.size();

    // TODO hahh
    assert(size >= acados.N + 1);
    // rclcpp print size and N
    for (int i = 0; i < acados.N; i++)
    {
        acados.setReference(refs.points[i].point.data(),
                            &refs.points[i].cz_prediction, i);
    }
    acados.setReference(refs.points[acados.N].point.data(), acados.N);

    solve();
}

void NMPC::stateCallback(const StateMsg& msg)
{
    t_start_ = std::chrono::high_resolution_clock::now();
    // msg.mpc_state.data();
    // copy states and controls to x_cur_
    std::copy(msg.mpc_state.data(), msg.mpc_state.data() + state_size,
              x_cur_.begin());
    std::copy(u_.data(), u_.data() + controls_size,
              x_cur_.begin() + state_size);
    acados.setState(const_cast<double*>(x_cur_.data()));

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    ready_pub_->publish(ready_msg);
}

NMPC::NMPC() : Node("NMPC"), file("time_record.csv")
{
    // Initialize the reference
    this->declare_parameter("mass", 1.0);
    this->get_parameter("mass", mass_);
    RCLCPP_INFO(this->get_logger(), "mass: %f", mass_);
    initSubsPubs();
    time_statistic.reserve(10000);
    u_.fill(9.8 / 4 * mass_);
    // NMPC setup code here (not shown, specific to your application)
}

NMPC::~NMPC()
{
    Eigen::VectorXd time_record;
    time_record.resize(time_statistic.size());
    for (size_t i = 0; i < time_statistic.size(); i++)
    {
        time_record[i] = time_statistic[i];
    }
    // save the time record to a csv file
    Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, "",
                              "\n");
    if (file.is_open())
    {
        file << time_record.format(CSVFormat);
        file.close();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open file");
    }
}

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::debug);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NMPC>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
