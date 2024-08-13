
#include <array>
#include <cassert>
#include <cfloat>
#include <cstddef>
#include <interface/msg/detail/trajectory_point__struct.hpp>
#include <rclcpp/logging.hpp>
#include <traj_buffer/traj_buffer.hpp>

#include "calculate_cz.cpp"

TrajectoryBuffer::TrajectoryBuffer() : Node("trajectory_buffer")
{
    initSubsPubs();
    declareParameters();
    updateParameters();
}

// state callback is to update cz_predict
void TrajectoryBuffer::stateCallback(const interface::msg::State::SharedPtr msg)
{
    auto state = msg->mpc_state;  // p,v,r,q
    // get v and q to eigen
    auto v = Eigen::Vector3d(state[3], state[4], state[5]);

    double v_norm = v.norm();
    // if (v_norm > last_v_norm_ + 0.5)
    // {
    discount = 1.0;

    if (v_norm < 0.5)
    {
        cz_predict_ = 0.24;
        return;
    }
    auto q = Eigen::Quaterniond(state[9], state[10], state[11], state[12]);
    Eigen::Vector3d vb = q.toRotationMatrix().transpose() * v;

    cz_predict_ = get_cz_from_vb(vb);

    std::cerr << "cz_predict_ " << cz_predict_ << std::endl;
    RCLCPP_INFO(this->get_logger(), "Entering ref_buffer state cz %f",
                cz_predict_);
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
            traj_bullet_pub_->publish(msg_fixed_point_);
        }
        else
        {
            if (index_ == trajectory_.points.size())
            {
                auto traj_last_point = trajectory_.points.back();
                for (auto& point : msg_fixed_point_.points)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        point.point[TrajecotryPoint::POS_START_INDEX + i] =
                            traj_last_point
                                .point[TrajecotryPoint::POS_START_INDEX + i];
                        point.point[TrajecotryPoint::VEL_START_INDEX + i]  = 0;
                        point.point[TrajecotryPoint::RATE_START_INDEX + i] = 0;
                    }
                    point.point[TrajecotryPoint::YB_START_INDEX]     = 0.0;
                    point.point[TrajecotryPoint::YB_START_INDEX + 1] = 1.0;
                    point.point[TrajecotryPoint::YB_START_INDEX]     = 0.0;
                }
                index_ = 0;
                traj_bullet_pub_->publish(msg_fixed_point_);
                traj_type_ = RefTrajType::FIXED;
            }
            RCLCPP_INFO(this->get_logger(), "refTrajectoryBulletPublish");
            RCLCPP_INFO(this->get_logger(), "index_ %d",
                        static_cast<int>(index_));
            auto   vis_msg      = visualization_msgs::msg::MarkerArray();
            double now_discount = 1.0;
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
                now_discount = discount;
                msg_planning_traj_.points[k].cz_prediction =
                    now_discount * cz_predict_;

                // auto marker            = visualization_msgs::msg::Marker();
                // marker.header.frame_id = "world";
                // marker.header.stamp    = this->now();
                // marker.action          =
                // visualization_msgs::msg::Marker::ADD; marker.type =
                // visualization_msgs::msg::Marker::ARROW; marker.scale.x = 1.0;
                // marker.scale.y         = 0.1;
                // marker.scale.z         = 0.1;
                // marker.color.a         = 0.2;
                // marker.color.r         = 0.0;
                // marker.color.g         = 0.0;
                // marker.color.b         = 1.0;
                // marker.pose.position.x =
                //     msg_planning_traj_.points[i - index_].point[0];
                // marker.pose.position.y =
                //     msg_planning_traj_.points[i - index_].point[1];
                // marker.pose.position.z =
                //     msg_planning_traj_.points[i - index_].point[2];
                // marker.id = i - index_;
                // marker.ns = "ref_bullet";
                // auto yb   = Eigen::Vector3d();
                // yb(0)     = msg_planning_traj_.points[i - index_]
                //             .point[TrajecotryPoint::YB_START_INDEX];
                // yb(1) = msg_planning_traj_.points[i - index_]
                //             .point[TrajecotryPoint::YB_START_INDEX + 1];
                // yb(2) = msg_planning_traj_.points[i - index_]
                //             .point[TrajecotryPoint::YB_START_INDEX + 2];

                // auto rot_axis  = Eigen::Vector3d::UnitX().cross(yb);
                // auto rot_angle = std::acos(Eigen::Vector3d::UnitX().dot(yb));
                // // from rot axis and angle to construct quaternion
                // Eigen::Quaterniond q;
                // q = Eigen::AngleAxisd(rot_angle, rot_axis.normalized());
                // marker.pose.orientation.x = q.x();
                // marker.pose.orientation.y = q.y();
                // marker.pose.orientation.z = q.z();
                // marker.pose.orientation.w = q.w();
                // vis_msg.markers.push_back(marker);
            }
            index_ += 1;
            traj_bullet_pub_->publish(msg_planning_traj_);
            ref_bullet_vis->publish(vis_msg);
        }
    }
};

void TrajectoryBuffer::updateTrajCallback(const Trajectory::SharedPtr msg)
{
    static std::array<double, 3> init_pos = {0.0, 0.0, 0.0};
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
        init_traj_set_ = true;
        RCLCPP_INFO(this->get_logger(), "trajectory_ size %zu",
                    trajectory_.points.size());
    }
    // print trajectory. point is an array {p[0], p[1], p[2], v[0], v[1], v[2],
    // r[0], r[1], r[2]}

#if !defined(BUILD_TYPE) || BUILD_TYPE != RELEASE
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
#endif
    // Trajectory is received, waiitng for ref_can_set to be true .
    // Warning Print that message.
    RCLCPP_WARN(this->get_logger(),
                "Trajectory is received, waiitng for /ref_can_set to be true");
};

void TrajectoryBuffer::declareParameters()
{
    this->declare_parameter<std::vector<double>>("pos", fixed_point_pos_);
    this->declare_parameter<std::vector<double>>("quat", fixed_point_quat_);
    this->declare_parameter<double>("traj_sampling_time", traj_sampling_time_);
    this->declare_parameter<double>("traj_control_time", traj_control_time_);
    this->declare_parameter<int>("down_sampling_factor", down_sampling_factor);
};

void TrajectoryBuffer::updateParameters()
{
    this->get_parameter("pos", fixed_point_pos_);
    this->get_parameter("quat", fixed_point_quat_);
    this->get_parameter("traj_sampling_time", traj_sampling_time_);
    this->get_parameter("traj_control_time", traj_control_time_);
    this->get_parameter("down_sampling_factor", down_sampling_factor);
    assert(std::abs(down_sampling_factor * traj_sampling_time_ -
                    traj_control_time_) < 1e-9);
    quat_eigen_ =
        Eigen::Quaterniond(fixed_point_quat_[0], fixed_point_quat_[1],
                           fixed_point_quat_[2], fixed_point_quat_[3]);
    quat_eigen_.normalize();
    RCLCPP_INFO(this->get_logger(), "FixedPoint Pos: [%f, %f, %f]",
                fixed_point_pos_[0], fixed_point_pos_[1], fixed_point_pos_[2]);
    RCLCPP_INFO(this->get_logger(),
                "FixedPoint Quat (w,x,y,z): [%f, %f, %f, %f]",
                fixed_point_quat_[0], fixed_point_quat_[1],
                fixed_point_quat_[2], fixed_point_quat_[3]);
    // Reminder: only w,z is allowed to adjusted. That is only yaw operates.
    if (fixed_point_quat_[0] >= 0 && abs(fixed_point_quat_[1]) <= FLT_EPSILON &&
        abs(fixed_point_quat_[2]) <= FLT_EPSILON)
    {
        yb_eigen_ = quat_eigen_.toRotationMatrix().col(1);
        auto yaw  = 2 * std::atan2(quat_eigen_.z(), quat_eigen_.w());
        RCLCPP_INFO(this->get_logger(), "Yb: [%f, %f, %f]", yb_eigen_[0],
                    yb_eigen_[1], yb_eigen_[2]);
        RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Ensuring w >=0");
        RCLCPP_ERROR(this->get_logger(), "Only yaw is allowed to be adjusted");
        std::exit(1);
    }
    for (auto& point : msg_fixed_point_.points)
    {
        point.point[0] = fixed_point_pos_[0];
        point.point[1] = fixed_point_pos_[1];
        point.point[2] = fixed_point_pos_[2];

        point.point[kYbStartIndex]     = yb_eigen_[0];
        point.point[kYbStartIndex + 1] = yb_eigen_[1];
        point.point[kYbStartIndex + 2] = yb_eigen_[2];
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
