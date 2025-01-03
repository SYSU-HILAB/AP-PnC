#include <chrono>
#include <cmath>
#include <cstddef>
#include <gcopter/gcopter.hpp>
#include <gcopter/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/trajectory_control.hpp>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
using std::chrono_literals::operator""s;
using RefTrajectory   = interface::msg::RefTrajectory;
using TrajectoryPoint = interface::msg::TrajectoryPoint;

#include <basic_trajectories/circle.hpp>
#include <basic_trajectories/lemniscate.hpp>
#include <basic_trajectories/line.hpp>
#include <basic_trajectories/sin.hpp>

std::ofstream &init_planning_csv_file()
{
    static std::ofstream csv_file;
    csv_file.open("planning.csv", std::ios::out | std::ios::trunc);
    // header t, p, v, thrust, alpha, aero_force_z  with header
    // name
    csv_file << "time"
             << ","
             << "pos_x"
             << ","
             << "pos_y"
             << ","
             << "pos_z"
             << ","
             << "vel_x"
             << ","
             << "vel_y"
             << ","
             << "vel_z"
             << ","
             << "thrust"
             << ","
             << "alpha"
             << ","
             << "aero_force_z"
             << ","
             << "yb_x"
             << ","
             << "yb_y"
             << ","
             << "yb_z"
             << ","
             << "av_x"
             << ","
             << "av_y"
             << ","
             << "av_z"
             << ","
             << "av_x_S"
             << ","
             << "av_z_S"
             << ","
             << "B_Omega_b_y"
             << ","
             << "alpha_dot"
             << ","
             << "B_aeroForce_z"
             << "\n";
    return csv_file;
}

// create a enum class for the basic trajectory type
enum class BasicTrajectoryType
{
    LEMNISCATE = 1,
    CIRCLE     = 2,
    SIN        = 3,
    LINE       = 4
};

struct Config
{
    double smoothingEps;
    int    integralIntervs;
    double relCostTol;
    double uavScale;
    // trajectory publish time resolution;
    double              traj_pub_rsl = 0.02;
    std::string         trajTopic    = "control_traj";
    std::vector<double> finVel;
    int pieceN = 20;  // 20 piece meaning 21 points including start and end, and
                      // that is to say, only pieceN - 1 control points
    int controlPoints = pieceN - 1;

    double box_side_inside_length  = 1;
    double box_side_outside_length = 1;
    double box_up_length           = 1;
    double box_down_length         = 0.1;
    double box_front_length        = 1;
    double box_back_length         = 1;

    std::string frame = "world";
    // basic trajectory set
    double basic_trajectory_rate             = 0.8;
    double basic_trajectory_radius           = 10;
    double basic_trajectory_height           = 5;
    int    basic_trajectory_type             = 1;
    double basic_trajectory_curvature_factor = 0.0;
    double basic_traj_wave_len               = 10.0;

    double trajecotry_cirled = true;

    gcopter::CostConfig &cost_config;

    Config(rclcpp::Node *node) : cost_config(gcopter::CostConfig::getInstance())
    {
        frame = node->declare_parameter("Frame", "world");

        uavScale = node->declare_parameter("UavScale", 1.0);

        smoothingEps    = node->declare_parameter("SmoothingEps", 1e-2);
        integralIntervs = node->declare_parameter("IntegralIntervs", 16);
        relCostTol      = node->declare_parameter("RelCostTol", 1e-4);
        pieceN          = node->declare_parameter("PieceN", 10);

        box_side_inside_length =
            node->declare_parameter("BoxSideInsideLength", 0.1);
        box_side_outside_length =
            node->declare_parameter("BoxSideOutsideLength", 1.0);
        box_down_length  = node->declare_parameter("BoxDownLength", 0.1);
        box_up_length    = node->declare_parameter("BoxUpLength", 0.5);
        box_front_length = node->declare_parameter("BoxFrontLength", 1.5);
        box_back_length  = node->declare_parameter("BoxBackLength", 1.5);

        basic_trajectory_rate =
            node->declare_parameter("BasicTrajectoryRate", 0.8);
        basic_trajectory_radius =
            node->declare_parameter("BasicTrajectoryRadius", 10.0);
        basic_trajectory_height =
            node->declare_parameter("BasicTrajectoryHeight", 5.0);
        basic_trajectory_type =
            node->declare_parameter("BasicTrajectoryType", 1);
        basic_trajectory_curvature_factor =
            node->declare_parameter("BasicTrajectoryCurvatureFactor", 0.0);
        basic_traj_wave_len =
            node->declare_parameter("BasicTrajectoryWaveLen", 10.0);

        trajecotry_cirled =
            node->declare_parameter("BasicTrajectoryCirled", true);

        // pub config
        traj_pub_rsl = node->declare_parameter("traj_pub_rsl", 0.02);
        trajTopic    = node->declare_parameter("trajTopic", "control_traj");
        // cost_config
        cost_config.v_max     = node->declare_parameter("v_max", 10.0);
        cost_config.omg_x_max = node->declare_parameter("omg_x_max", 3.0);
        cost_config.omg_y_max = node->declare_parameter("omg_y_max", 3.0);
        cost_config.omg_z_max = node->declare_parameter("omg_z_max", 3.0);

        cost_config.thrust_max = node->declare_parameter("thrust_max", 20.0);
        cost_config.thrust_min = node->declare_parameter("thrust_min", 1.0);
        cost_config.acc_max    = node->declare_parameter("acc_max", 1.0);

        cost_config.thrust_weight =
            node->declare_parameter("thrust_weight", 1e5);
        cost_config.vel_weight   = node->declare_parameter("vel_weight", 1e4);
        cost_config.acc_weight   = node->declare_parameter("acc_weight", 1e4);
        cost_config.omg_x_weight = node->declare_parameter("omg_x_weight", 1e3);
        cost_config.omg_y_weight = node->declare_parameter("omg_y_weight", 1e3);
        cost_config.omg_z_weight = node->declare_parameter("omg_z_weight", 1e3);
        cost_config.time_weight =
            node->declare_parameter("time_weight", 1e2 * 5);
        cost_config.omg_consistent_weight =
            node->declare_parameter("omg_consistent_weight", 1e3);
    }

    void UpdateConfig(const rclcpp::Node *node)
    {
        node->get_parameter("SmoothingEps", smoothingEps);
        node->get_parameter("IntegralIntervs", integralIntervs);
        node->get_parameter("RelCostTol", relCostTol);
        node->get_parameter("UavScale", uavScale);

        node->get_parameter("FinVel", finVel);
        node->get_parameter("PieceN", pieceN);

        node->get_parameter("BoxSideInsideLength", box_side_inside_length);
        node->get_parameter("BoxSideOutsideLength", box_side_outside_length);
        node->get_parameter("BoxUpLength", box_up_length);
        node->get_parameter("BoxDownLength", box_down_length);
        node->get_parameter("BoxFrontLength", box_front_length);
        node->get_parameter("BoxBackLength", box_back_length);

        node->get_parameter("Frame", frame);

        node->get_parameter("BasicTrajectoryRate", basic_trajectory_rate);
        node->get_parameter("BasicTrajectoryRadius", basic_trajectory_radius);
        node->get_parameter("BasicTrajectoryHeight", basic_trajectory_height);
        node->get_parameter("BasicTrajectoryType", basic_trajectory_type);
        node->get_parameter("BasicTrajectoryCurvatureFactor",
                            basic_trajectory_curvature_factor);
        node->get_parameter("BasicTrajectoryWaveLen", basic_traj_wave_len);
        node->get_parameter("TrajectoryCirled", trajecotry_cirled);
        // update cost_config
        node->get_parameter("v_max", cost_config.v_max);
        node->get_parameter("omg_x_max", cost_config.omg_x_max);
        node->get_parameter("omg_y_max", cost_config.omg_y_max);
        node->get_parameter("omg_z_max", cost_config.omg_z_max);
        node->get_parameter("thrust_max", cost_config.thrust_max);
        node->get_parameter("thrust_min", cost_config.thrust_min);
        node->get_parameter("acc_max", cost_config.acc_max);
        node->get_parameter("thrust_weight", cost_config.thrust_weight);
        node->get_parameter("vel_weight", cost_config.vel_weight);
        node->get_parameter("omg_x_weight", cost_config.omg_x_weight);
        node->get_parameter("omg_y_weight", cost_config.omg_y_weight);
        node->get_parameter("omg_z_weight", cost_config.omg_z_weight);
        node->get_parameter("time_weight", cost_config.time_weight);
        node->get_parameter("acc_weight", cost_config.acc_weight);
        node->get_parameter("omg_consistent_weight",
                            cost_config.omg_consistent_weight);

        node->get_parameter("traj_pub_rsl", traj_pub_rsl);
        node->get_parameter("trajTopic", trajTopic);
    }
};

class GlobalPlanner : public rclcpp::Node
{
   private:
    Config                                                           config;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
    // Visualizer visualizer;
    RefTrajectory                                   control_traj_msg;
    Trajectory<5>                                   traj;
    double                                          trajStamp;
    std::unique_ptr<basic_trajectories::Trajectory> basic_traj;
    rclcpp::TimerBase::SharedPtr                    timer;
    rclcpp::Publisher<RefTrajectory>::SharedPtr     trajPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr trajVizPub;

    bool init_setup = false;

   public:
    GlobalPlanner() : Node("global_planning_node"), config(this)
    {
        config.UpdateConfig(this);
        auto reliable_qos =
            rclcpp::QoS(10).transient_local().reliable().keep_last(1);
        trajVizPub = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "traj_viz", reliable_qos);

        trajPub = this->create_publisher<RefTrajectory>(config.trajTopic,
                                                        reliable_qos);

        init_basic_trajectory_type(config.basic_trajectory_type);
        timer =
            this->create_wall_timer(2s, std::bind(&GlobalPlanner::plan, this));

        RCLCPP_INFO(this->get_logger(), "Planning");
    }

    void init_basic_trajectory_type(const int type)
    {
        // basic_trajniscate
        if (type == 1)
        {
            basic_traj = std::make_unique<basic_trajectories::Lemniscate>(
                config.basic_trajectory_radius, config.basic_trajectory_rate,
                config.basic_trajectory_height,
                config.basic_trajectory_curvature_factor);
            return;
        }
        if (type == 2)
        {
            basic_traj = std::make_unique<basic_trajectories::Circle>(
                config.basic_trajectory_radius, config.basic_trajectory_rate,
                config.basic_trajectory_height);
            return;
        }
        if (type == 3)
        {
            basic_traj = std::make_unique<basic_trajectories::Sin>(
                config.basic_trajectory_radius, config.basic_traj_wave_len,
                config.basic_trajectory_height);
            config.basic_trajectory_rate = basic_traj->getAngularSpeed();
            return;
        }
        if (type == 4)
        {
            if (!config.trajecotry_cirled)
            {
                basic_traj = std::make_unique<basic_trajectories::Line>(
                    config.basic_trajectory_radius,
                    config.basic_trajectory_rate,
                    config.basic_trajectory_height);
                return;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),
                             "The line trajectory must not be cirled!");
                abort();
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                         "The trajecotry type is not supported!");
        }
    }
    void create_basic_trajecotry_sfc(std::vector<Eigen::MatrixX4d> &hPolys)
    {
        Eigen::RowVector4d h1, h2, h3, h4, h5, h6;

        Eigen::MatrixX4d H;
        H.resize(6, 4);

        std::cerr << "H\n" << H << std::endl;
        auto route = basic_traj->samplingPointArray(config.pieceN);

        auto traj_vis_msg            = basic_traj->toMsg();
        traj_vis_msg.header.frame_id = config.frame;
        traj_vis_msg.header.stamp    = this->get_clock()->now();
        this->trajVizPub->publish(traj_vis_msg);

        Eigen::Vector3d pc, v_direction, u_direction;
        for (size_t i = 1; i < route.size() - 1; i++)
        {
            auto point  = route[i];
            pc          = point.col(0);
            v_direction = point.col(1);
            v_direction.normalize();
            u_direction =
                (Eigen::Vector3d() << -v_direction(1), v_direction(0), 0)
                    .finished();
            u_direction.normalize();
            if (pc[0] >= 0)
            {
                // u_direction = -u_direction;
                u_direction = -u_direction;
            }
            h1 << v_direction.transpose(),
                -v_direction.dot(pc) - config.box_front_length;
            h2 << -v_direction.transpose(),
                v_direction.dot(pc) - config.box_back_length;
            h3 << u_direction.transpose(),
                -u_direction.dot(pc) - config.box_side_outside_length;
            h4 << -u_direction.transpose(),
                u_direction.dot(pc) - config.box_side_inside_length;
            h5 << 0, 0, 1, -pc.z() - config.box_up_length;
            h6 << 0, 0, -1, pc.z() - config.box_down_length;
            H << h1, h2, h3, h4, h5, h6;
            hPolys.push_back(H);
        }
    }

    inline void plan()
    {
        if (init_setup)
        {
            return;
        }
        std::vector<Eigen::MatrixX4d> hPolys;
        RCLCPP_INFO(this->get_logger(), "Planning");
        create_basic_trajecotry_sfc(hPolys);

        Eigen::Matrix3d iniState, finState;
        auto            route = basic_traj->samplingPointArray(config.pieceN);
        iniState << route.front().col(0), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero();
        finState << route.back().col(0), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero();
        gcopter::GCOPTER_PolytopeSFC gcopter;

        traj.clear();
        Eigen::VectorXd initTime;
        initTime.resize(config.pieceN);
        initTime.setConstant(2 * M_PI / config.basic_trajectory_rate /
                             config.pieceN);

        Eigen::Matrix3Xd initPointsPos(3, config.pieceN - 1);
        for (int i = 0; i < config.pieceN - 1; i++)
        {
            Eigen::Vector3d v = route[i].col(1);
            v.normalize();
            Eigen::Vector3d u = v.cross(Eigen::Vector3d::UnitZ());
            initPointsPos.col(i) =
                route[i + 1].col(0) + u * config.box_side_outside_length * 0.8;
        }

        // print initTime and init Pos
        std::cerr << "initTime: " << initTime.transpose() << std::endl;
        std::cerr << "initPointsPos: \n" << initPointsPos << std::endl;
        // print size of time and pos
        std::cerr << "initTime size: " << initTime.size() << std::endl;
        std::cerr << "initPointsPos size: " << initPointsPos.size()
                  << std::endl;
        // print end point
        std::cerr << "finState: \n" << finState << std::endl;
        if (!init_setup)
        {
            if (!gcopter.setup_basic_trajectory(
                    iniState, finState, initTime, initPointsPos, hPolys,
                    config.smoothingEps, config.integralIntervs))
            {
                return;
            }
            if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
            {
                return;
            }
            init_setup = true;

            auto route_msg            = basic_traj->toMsg(100);
            route_msg.header.frame_id = config.frame;
            route_msg.header.stamp    = this->get_clock()->now();
            this->trajVizPub->publish(route_msg);

            auto &tailsitter_data = tailsitter_df::Data::getInstance();
            tailsitter_data.reset(config.trajecotry_cirled);
            control_traj_msg.points.clear();
            auto &csv_file = init_planning_csv_file();
            for (double t = 0.0; t <= traj.getTotalDuration() + FLT_EPSILON;
                 t += config.traj_pub_rsl)
            {
                // get model attitude and thrust and omg
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;
                double          thrust;

                tailsitter_data.df_backward_auto(traj.getVel(t), traj.getAcc(t),
                                                 traj.getAcc(t), thrust, omg);

                {
                    // Get all required data from tailsitter model
                    const auto &pos = traj.getPos(t);
                    const auto &vel = traj.getVel(t);
                    const auto &acc = traj.getAcc(t);
                    const auto &yb  = tailsitter_data.getYb();

                    // Calculate acceleration without gravity
                    const Eigen::Vector3d gravity(0, 0, 9.81);
                    const Eigen::Vector3d av = acc - gravity;

                    // Get aerodynamic data
                    const double alpha     = tailsitter_data.getAlpha();
                    const double alpha_dot = tailsitter_data.getAlphaDot();
                    const double aero_force_z =
                        tailsitter_data.getAeroForceB()(2);
                    const double B_aeroForce_z =
                        tailsitter_data.getBAeroForceZ();
                    const double av_x_S      = tailsitter_data.getSAvX();
                    const double av_z_S      = tailsitter_data.getSAvZ();
                    const double B_Omega_b_y = tailsitter_data.getBOmegaY();

                    // Write data to CSV in consistent order
                    csv_file << t << "," << pos(0) << "," << pos(1) << ","
                             << pos(2) << "," << vel(0) << "," << vel(1) << ","
                             << vel(2) << "," << thrust << "," << alpha << ","
                             << aero_force_z << "," << yb(0) << "," << yb(1)
                             << "," << -yb(2) << "," << av(0) << "," << av(1)
                             << "," << av(2) << "," << av_x_S << "," << av_z_S
                             << "," << B_Omega_b_y << "," << alpha_dot << ","
                             << B_aeroForce_z << "\n";
                }

                auto               quat_df = tailsitter_data.getQuaternion();
                Eigen::Quaterniond q_b1_b2(0, 0.707, 0, 0.707);
                auto               quat_q = quat_df * q_b1_b2;

                quat(0) = quat_q.w();
                quat(1) = quat_q.x();
                quat(2) = quat_q.y();
                quat(3) = quat_q.z();
                if (quat(0) < 0 - FLT_EPSILON)
                {
                    quat = -quat;
                }

                auto p = traj.getPos(t);
                auto v = traj.getVel(t);

                auto            yb_df = tailsitter_data.getYb();
                Eigen::Vector3d YB =
                    (Eigen::Vector3d() << -yb_df(0), -yb_df(1), yb_df(2))
                        .finished();

                control_traj_msg.timestamp = this->get_clock()->now();
                auto traj_point            = TrajectoryPoint();
                traj_point.point[TrajectoryPoint::POS_START_INDEX]     = p[0];
                traj_point.point[TrajectoryPoint::POS_START_INDEX + 1] = p[1];
                traj_point.point[TrajectoryPoint::POS_START_INDEX + 2] = p[2];
                traj_point.point[TrajectoryPoint::VEL_START_INDEX]     = v[0];
                traj_point.point[TrajectoryPoint::VEL_START_INDEX + 1] = v[1];
                traj_point.point[TrajectoryPoint::VEL_START_INDEX + 2] = v[2];
                traj_point.point[TrajectoryPoint::YB_START_INDEX]      = YB[0];
                traj_point.point[TrajectoryPoint::YB_START_INDEX + 1]  = YB[1];
                traj_point.point[TrajectoryPoint::YB_START_INDEX + 2]  = YB[2];
                // default set to zero
                control_traj_msg.points.push_back(traj_point);
            }

            trajPub->publish(control_traj_msg);
            RCLCPP_WARN(this->get_logger(), "Trajectory Published !!!\n");
            std::vector<Eigen::Vector3d> route_path;
            for (int i = 0; i < config.pieceN; i++)
            {
                route_path.emplace_back(route[i].col(0));
            }
            if (traj.getPieceNum() > 0)
            {
                trajStamp = this->get_clock()->now().seconds();
            }
            // clsoe csv file
            csv_file.close();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto global_planner = std::make_shared<GlobalPlanner>();
    rclcpp::spin(global_planner);
    rclcpp::shutdown();

    return 0;
}
