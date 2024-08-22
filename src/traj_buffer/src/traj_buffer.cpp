
#include <array>
#include <cassert>
#include <cfloat>
#include <cstddef>
#include <interface/msg/detail/trajectory_point__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_buffer/aerodyn.hpp>
#include <traj_buffer/traj_buffer.hpp>

TrajectoryBuffer::TrajectoryBuffer() : Node("trajectory_buffer")
{
    initSubsPubs();
    declareParameters();
    updateParameters();
}

bool TrajectoryBuffer::checkTrajectoryTermination()
{
    bool ifTerminated = (index_ == trajectory_.points.size()) ? true : false;
    if (ifTerminated)
    {
        RCLCPP_WARN(this->get_logger(), "Trajectory is terminated");
        auto traj_last_point = trajectory_.points.back();
        for (auto& point : msg_fixed_point_.points)
        {
            for (int i = 0; i < 3; i++)
            {
                point.point[TrajecotryPoint::POS_START_INDEX + i] =
                    traj_last_point.point[TrajecotryPoint::POS_START_INDEX + i];
                point.point[TrajecotryPoint::VEL_START_INDEX + i]  = 0;
                point.point[TrajecotryPoint::RATE_START_INDEX + i] = 0;
            }
            point.point[TrajecotryPoint::YB_START_INDEX] =
                traj_last_point.point[TrajecotryPoint::YB_START_INDEX];
            point.point[TrajecotryPoint::YB_START_INDEX + 1] =
                traj_last_point.point[TrajecotryPoint::YB_START_INDEX + 1];
            point.point[TrajecotryPoint::YB_START_INDEX] =
                traj_last_point.point[TrajecotryPoint::YB_START_INDEX + 2];
        }

        if (circled_)
        {
            traj_type_ = RefTrajType::PLANNING;
            index_     = 0;
        }
        else
        {
            traj_type_ = RefTrajType::FIXED;
        }
    }
    return ifTerminated;
}
// state callback is to update cz_predict
void TrajectoryBuffer::stateCallback(const interface::msg::State::SharedPtr msg)
{
    auto state = msg->mpc_state;  // p,v,r,q
    // get v and q to eigen
    auto v = Eigen::Vector3d(state[3], state[4], state[5]);
    auto q = Eigen::Quaterniond(state[9], state[10], state[11], state[12]);
    Eigen::Vector3d vb = q.toRotationMatrix().transpose() * v;

    if (real_world_)
    {
        cz_predict_ = aero_dyn_.getAeroCzVb(
            vb, aerodyn::BsplineAeroDyn::AeroDynType::Exp);
    }
    else
    {
        cz_predict_ = aero_dyn_.getAeroCzVb(
            vb, aerodyn::BsplineAeroDyn::AeroDynType::Sim);
    }
    static bool first = true;
    if (first)
    {
        if (real_world_)
            RCLCPP_WARN(this->get_logger(), "Caution: Real World Mode");
        RCLCPP_WARN(this->get_logger(), "cz_predict_ %f", cz_predict_);
        first = false;
    }
};

void TrajectoryBuffer::refTrajectoryBulletPublish(
    std_msgs::msg::Bool::SharedPtr msg)
{
    static bool first_time = true;

    if (msg->data)
    {
        if (traj_type_ == RefTrajType::FIXED)
        {
            if (first_time)
            {
                RCLCPP_WARN(this->get_logger(), "Fixed Point Publish");
                first_time = false;
            }
            tracking_valid_                 = false;
            msg_fixed_point_.tracking_valid = tracking_valid_;

            traj_bullet_pub_->publish(msg_fixed_point_);
        }

        // traj_type_ == RefTrajType::PLANNING
        else
        {
            RCLCPP_INFO(this->get_logger(), "refTrajectoryBulletPublish");
            RCLCPP_INFO(this->get_logger(), "index_ %d",
                        static_cast<int>(index_));
            if (checkTrajectoryTermination())
            {
                return;
            }
            tracking_valid_                   = true;
            msg_planning_traj_.tracking_valid = tracking_valid_;

            // Leaving all cz prediction to a seperate function
            for (size_t k = 0; k < kRefTrajBulletSize; k++)
            {
                if (index_ + down_sampling_factor * k <
                    trajectory_.points.size())
                {
                    msg_planning_traj_.points[k] =
                        trajectory_.points[index_ + down_sampling_factor * k];
                }
                else
                {
                    msg_planning_traj_.points[k] = trajectory_.points.back();
                }
                msg_planning_traj_.points[k].cz_prediction = cz_predict_;
            }
            index_ += 1;
            traj_bullet_pub_->publish(msg_planning_traj_);
        }
    }
};

void TrajectoryBuffer::updateTrajCallback(const Trajectory::SharedPtr msg)
{
    static std::array<double, 3> init_pos = {0.0, 0.0, 0.0};
    static std::array<double, 3> init_yb  = {0.0, 1.0, 0.0};
    std::cerr << "updateTrajCallback" << std::endl;
    if (init_traj_set_ == false)
    {
        trajectory_.points.clear();
        // copy msg to trajectory_
        trajectory_.points = msg->points;
        init_pos[0] =
            trajectory_.points[0]
                .point[interface::msg::TrajectoryPoint::POS_START_INDEX];
        init_pos[1] =
            trajectory_.points[0]
                .point[interface::msg::TrajectoryPoint::POS_START_INDEX + 1];

        // change ref_thrust to gravity force.
        for (auto& point : trajectory_.points)
        {
            point.point[TrajecotryPoint::POS_START_INDEX] -= init_pos[0];
            point.point[TrajecotryPoint::POS_START_INDEX + 1] -= init_pos[1];
            point.point[TrajecotryPoint::THR_START_INDEX]     = kThrustPerRotor;
            point.point[TrajecotryPoint::THR_START_INDEX + 1] = kThrustPerRotor;
            point.point[TrajecotryPoint::THR_START_INDEX + 2] = kThrustPerRotor;
            point.point[TrajecotryPoint::THR_START_INDEX + 3] = kThrustPerRotor;
        }
        for (int i = 0; i < 3; i++)
        {
            init_yb[i] =
                trajectory_.points[0]
                    .point[interface::msg::TrajectoryPoint::YB_START_INDEX + i];
        }
        for (auto& point : msg_fixed_point_.points)
        {
            point.point[TrajecotryPoint::POS_START_INDEX] =
                trajectory_.points[0].point[TrajecotryPoint::POS_START_INDEX];
            point.point[TrajecotryPoint::POS_START_INDEX + 1] =
                trajectory_.points[0]
                    .point[TrajecotryPoint::POS_START_INDEX + 1];
            point.point[TrajecotryPoint::POS_START_INDEX + 2] =
                trajectory_.points[0]
                    .point[TrajecotryPoint::POS_START_INDEX + 2];
            point.point[TrajecotryPoint::YB_START_INDEX]     = init_yb[0];
            point.point[TrajecotryPoint::YB_START_INDEX + 1] = init_yb[1];
            point.point[TrajecotryPoint::YB_START_INDEX + 2] = init_yb[2];
        }

        init_traj_set_ = true;
    }

    RCLCPP_WARN(this->get_logger(), "Trajectory points size %zu",
                trajectory_.points.size());
    RCLCPP_WARN(this->get_logger(),
                "Trajectory is received, waiitng for /ref_can_set to be true");
};

void TrajectoryBuffer::declareParameters()
{
    this->declare_parameter<double>("traj_sampling_time", traj_sampling_time_);
    this->declare_parameter<double>("traj_control_time", traj_control_time_);
    this->declare_parameter<int>("down_sampling_factor", down_sampling_factor);
    this->declare_parameter<bool>("circled", circled_);
    this->declare_parameter<bool>("real_world", real_world_);
    this->declare_parameter<std::vector<double>>(
        "exp_coefs",
        {0.08869374325729838, 0.11713376801189856, 0.12567841873810845,
         0.15005373297969835, 0.1324185630496418, 0.24883612641861838,
         0.20757431730331458, 0.5001433323539398, 0.3877382239367887,
         0.08974898500006634, 0.14520286916801822, 0.08072293755821285});
    this->declare_parameter<std::vector<double>>(
        "sim_coefs",
        {-0.0008318947521944341, 0.19855584986410266, 0.16605095441849582,
         0.22746737306109233, 0.22377476306115018, 0.16675851490605675,
         0.17886373334939626, 0.07910326162993098});
};

void TrajectoryBuffer::updateParameters()
{
    this->get_parameter("traj_sampling_time", traj_sampling_time_);
    this->get_parameter("traj_control_time", traj_control_time_);
    this->get_parameter("down_sampling_factor", down_sampling_factor);
    this->get_parameter("circled", circled_);
    this->get_parameter("real_world", real_world_);
    this->get_parameter("exp_coefs", exp_coefs_);
    this->get_parameter("sim_coefs", sim_coefs_);
    assert(std::abs(down_sampling_factor * traj_sampling_time_ -
                    traj_control_time_) < 1e-9);
    aero_dyn_.setCoefs(exp_coefs_, aerodyn::BsplineAeroDyn::AeroDynType::Exp);
    aero_dyn_.setCoefs(sim_coefs_, aerodyn::BsplineAeroDyn::AeroDynType::Sim);
    // print circled, exp_coefs, sim_coefs
    RCLCPP_INFO(this->get_logger(), "circled: %d", circled_);
    RCLCPP_INFO(this->get_logger(), "exp_coefs: ");
    for (auto coef : exp_coefs_)
    {
        RCLCPP_INFO(this->get_logger(), "%f", coef);
    }
    RCLCPP_INFO(this->get_logger(), "sim_coefs: ");
    for (auto coef : sim_coefs_)
    {
        RCLCPP_INFO(this->get_logger(), "%f", coef);
    }
}

void TrajectoryBuffer::modeTriggerCallback(std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        if (traj_type_ == RefTrajType::FIXED)
        {
            traj_type_ = RefTrajType::PLANNING;
            index_     = 0;
            RCLCPP_WARN(this->get_logger(), "Switch to Planning Trajectory");
        }
        else
        {
            traj_type_ = RefTrajType::FIXED;
            RCLCPP_WARN(this->get_logger(), "Switch to Fixed Point Trajectory");
        }
    }
};

void TrajectoryBuffer::initSubsPubs()
{
    ref_can_set_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "ref_can_set", reliabe_qos_keep_last,
        std::bind(&TrajectoryBuffer::refTrajectoryBulletPublish, this,
                  std::placeholders::_1));
    mode_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "mode", reliabe_qos_keep_last,
        std::bind(&TrajectoryBuffer::modeTriggerCallback, this,
                  std::placeholders::_1));
    traj_sub_ = this->create_subscription<Trajectory>(
        "control_traj", reliabe_qos_keep_last,
        std::bind(&TrajectoryBuffer::updateTrajCallback, this,
                  std::placeholders::_1));

    state_sub_ = this->create_subscription<interface::msg::State>(
        "cur_state", reliabe_qos_keep_last,
        std::bind(&TrajectoryBuffer::stateCallback, this,
                  std::placeholders::_1));

    ref_bullet_vis =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "ref_bullet_vis", reliabe_qos_keep_last);

    traj_bullet_pub_ = this->create_publisher<TrajecotryBullet>(
        "ref_trajectory_bullet", reliabe_qos_keep_last);
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryBuffer>());
    rclcpp::shutdown();
    return 0;
}
