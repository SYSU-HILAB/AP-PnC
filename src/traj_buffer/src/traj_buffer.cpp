
#include <array>
#include <cassert>
#include <cfloat>
#include <cstddef>
#include <interface/msg/detail/trajectory_point__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_buffer/aerodyn.hpp>
#include <traj_buffer/traj_buffer.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

void push_to_vis(visualization_msgs::msg::MarkerArray& vis, Trajectory& traj)
{
    auto marker            = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.type            = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x         = 0.1;
    marker.scale.y         = 0.01;
    marker.scale.z         = 0.01;
    marker.color.a         = 0.2;
    marker.color.r         = 0.0;
    marker.color.g         = 0.0;
    marker.color.b         = 1.0;
    int index              = 0;
    for (auto point : traj.points)
    {
        marker.pose.position.x = point.point[TrajecotryPoint::POS_START_INDEX];
        marker.pose.position.y =
            point.point[TrajecotryPoint::POS_START_INDEX + 1];
        marker.pose.position.z =
            point.point[TrajecotryPoint::POS_START_INDEX + 2];
        vis.markers.push_back(marker);
        marker.id = index++;
        marker.ns = "ref_vis";
        auto yb   = Eigen::Vector3d();
        yb(0)     = point.point[TrajecotryPoint::YB_START_INDEX];
        yb(1)     = point.point[TrajecotryPoint::YB_START_INDEX + 1];
        yb(2)     = point.point[TrajecotryPoint::YB_START_INDEX + 2];

        auto rot_axis  = Eigen::Vector3d::UnitX().cross(yb);
        auto rot_angle = std::acos(Eigen::Vector3d::UnitX().dot(yb));
        // from rot axis and angle to construct quaternion
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(rot_angle, rot_axis.normalized());
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        vis.markers.push_back(marker);
    }
}

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
    auto state = msg->state;  // p,v,r,q
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

            auto v1 = Eigen::Vector3d(
                msg_planning_traj_.points[0]
                    .point[interface::msg::TrajectoryPoint::VEL_START_INDEX],
                msg_planning_traj_.points[0]
                    .point[interface::msg::TrajectoryPoint::VEL_START_INDEX +
                           1],
                msg_planning_traj_.points[0]
                    .point[interface::msg::TrajectoryPoint::VEL_START_INDEX +
                           2]);
            auto v2 = Eigen::Vector3d(
                msg_planning_traj_.points[1]
                    .point[interface::msg::TrajectoryPoint::VEL_START_INDEX],
                msg_planning_traj_.points[1]
                    .point[interface::msg::TrajectoryPoint::VEL_START_INDEX +
                           1],
                msg_planning_traj_.points[1]
                    .point[interface::msg::TrajectoryPoint::VEL_START_INDEX +
                           2]);

            if (v1.norm() < v2.norm() + 0.3)
            {
                for (auto point : msg_planning_traj_.points)
                {
                    point.cz_prediction *= acc_aero_fix_factor;
                }
            }
            if (v1.norm() > v2.norm() + 0.3)
            {
                for (auto point : msg_planning_traj_.points)
                {
                    point.cz_prediction *= deacc_aero_fix_factor;
                }
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
            point.point[TrajecotryPoint::POS_START_INDEX + 2] -= init_pos[2];
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

    static auto vis_msg = visualization_msgs::msg::MarkerArray();
    push_to_vis(vis_msg, trajectory_);
    ref_vis_pub_->publish(vis_msg);

    RCLCPP_WARN(this->get_logger(), "Trajectory points size %zu",
                trajectory_.points.size());
    RCLCPP_WARN(this->get_logger(),
                "Trajectory is received, waiitng for /ref_can_set to be true");
};

void TrajectoryBuffer::declareParameters()
{
    this->declare_parameter<double>("acc_aero_fix_factor", acc_aero_fix_factor);
    this->declare_parameter<double>("deacc_aero_fix_factor",
                                    deacc_aero_fix_factor);
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
    this->get_parameter("acc_aero_fix_factor", acc_aero_fix_factor);
    this->get_parameter("deacc_aero_fix_factor", deacc_aero_fix_factor);
    assert(std::abs(down_sampling_factor * traj_sampling_time_ -
                    traj_control_time_) < 1e-9);
    aero_dyn_.setCoefs(exp_coefs_, aerodyn::BsplineAeroDyn::AeroDynType::Exp);
    aero_dyn_.setCoefs(sim_coefs_, aerodyn::BsplineAeroDyn::AeroDynType::Sim);
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

    ref_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "ref_vis", reliabe_qos_keep_last);

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
