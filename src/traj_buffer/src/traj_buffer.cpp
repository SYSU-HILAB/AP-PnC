
#include <array>
#include <cassert>
#include <cfloat>
#include <cstddef>
#include <interface/msg/detail/trajectory_point__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_buffer/traj_buffer.hpp>

#include "calculate_cz.cpp"

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

    double v_norm = v.norm();

    if (v_norm < 0.5)
    {
        cz_predict_ = 0.24;
        return;
    }
    auto q = Eigen::Quaterniond(state[9], state[10], state[11], state[12]);
    Eigen::Vector3d vb = q.toRotationMatrix().transpose() * v;

    cz_predict_ = get_cz_from_vb(vb);
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
        RCLCPP_INFO(this->get_logger(), "trajectory_ size %zu",
                    trajectory_.points.size());
    }
    // print trajectory. point is an array {p[0], p[1], p[2], v[0], v[1],
    // v[2], r[0], r[1], r[2]}

    for (size_t i = 0; i < trajectory_.points.size(); i++)
    {
        RCLCPP_INFO(
            this->get_logger(), "point %zu: %f %f %f %f %f %f %f %f %f", i,
            trajectory_.points[i].point[0], trajectory_.points[i].point[1],
            trajectory_.points[i].point[2], trajectory_.points[i].point[3],
            trajectory_.points[i].point[4], trajectory_.points[i].point[5],
            trajectory_.points[i].point[6], trajectory_.points[i].point[7],
            trajectory_.points[i].point[8]);
    }
    // Trajectory is received, waiitng for ref_can_set to be true .
    // Warning Print that message.
    RCLCPP_WARN(this->get_logger(),
                "Trajectory is received, waiitng for /ref_can_set to be true");
};

void TrajectoryBuffer::declareParameters()
{
    this->declare_parameter<double>("traj_sampling_time", traj_sampling_time_);
    this->declare_parameter<double>("traj_control_time", traj_control_time_);
    this->declare_parameter<int>("down_sampling_factor", down_sampling_factor);
    this->declare_parameter<bool>("circled", circled_);
};

void TrajectoryBuffer::updateParameters()
{
    this->get_parameter("traj_sampling_time", traj_sampling_time_);
    this->get_parameter("traj_control_time", traj_control_time_);
    this->get_parameter("down_sampling_factor", down_sampling_factor);
    this->get_parameter("circled", circled_);
    assert(std::abs(down_sampling_factor * traj_sampling_time_ -
                    traj_control_time_) < 1e-9);
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
