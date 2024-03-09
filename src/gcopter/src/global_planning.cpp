#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config {
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    double uavScale;

    Config(rclcpp::Node *node) {
        mapTopic = node->declare_parameter("MapTopic", "default_map_topic");
        targetTopic = node->declare_parameter("TargetTopic", "default_target_topic");
        dilateRadius = node->declare_parameter("DilateRadius", 1.0);
        voxelWidth = node->declare_parameter("VoxelWidth", 1.0);
        mapBound = node->declare_parameter("MapBound", std::vector<double>({-25.0, 25.0, -25.0, 25.0, 0.0, 5.0}));
        timeoutRRT = node->declare_parameter("TimeoutRRT", 1.0);
        maxVelMag = node->declare_parameter("MaxVelMag", 1.0);
        maxBdrMag = node->declare_parameter("MaxBdrMag", 1.0);
        maxTiltAngle = node->declare_parameter("MaxTiltAngle", 1.0);
        minThrust = node->declare_parameter("MinThrust", 1.0);
        maxThrust = node->declare_parameter("MaxThrust", 1.0);
        vehicleMass = node->declare_parameter("VehicleMass", 1.0);
        gravAcc = node->declare_parameter("GravAcc", 1.0);
        horizDrag = node->declare_parameter("HorizDrag", 1.0);
        vertDrag = node->declare_parameter("VertDrag", 1.0);
        parasDrag = node->declare_parameter("ParasDrag", 1.0);
        speedEps = node->declare_parameter("SpeedEps", 1.0);
        weightT = node->declare_parameter("WeightT", 1.0);
        uavScale = node->declare_parameter("UavScale", 1.0);
        chiVec = node->declare_parameter("ChiVec", std::vector<double>({1.0e+4, 1.0e+4, 1.0e+4, 1.0e+4, 1.0e+5}));
        smoothingEps = node->declare_parameter("SmoothingEps", 1.0);
        integralIntervs = node->declare_parameter("IntegralIntervs", 1);
        relCostTol = node->declare_parameter("RelCostTol", 1.0);

    }

    void UpdateConfig(const rclcpp::Node *node) {
        node->get_parameter("MapTopic", mapTopic);
        node->get_parameter("TargetTopic", targetTopic);
        node->get_parameter("DilateRadius", dilateRadius);
        node->get_parameter("VoxelWidth", voxelWidth);
        node->get_parameter("MapBound", mapBound);
        node->get_parameter("TimeoutRRT", timeoutRRT);
        node->get_parameter("MaxVelMag", maxVelMag);
        node->get_parameter("MaxBdrMag", maxBdrMag);
        node->get_parameter("MaxTiltAngle", maxTiltAngle);
        node->get_parameter("MinThrust", minThrust);
        node->get_parameter("MaxThrust", maxThrust);
        node->get_parameter("VehicleMass", vehicleMass);
        node->get_parameter("GravAcc", gravAcc);
        node->get_parameter("HorizDrag", horizDrag);
        node->get_parameter("VertDrag", vertDrag);
        node->get_parameter("ParasDrag", parasDrag);
        node->get_parameter("SpeedEps", speedEps);
        node->get_parameter("WeightT", weightT);
        node->get_parameter("ChiVec", chiVec);
        node->get_parameter("SmoothingEps", smoothingEps);
        node->get_parameter("IntegralIntervs", integralIntervs);
        node->get_parameter("RelCostTol", relCostTol);
        node->get_parameter("UavScale", uavScale);
    }
};

class GlobalPlanner : public rclcpp::Node {
private:
    Config config;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

public:
    GlobalPlanner()
            : Node("global_planning_node"),
              config(this),
              mapInitialized(false),
              visualizer(this) {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        config.UpdateConfig(this);


        mapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(config.mapTopic, 1,
                                                                          std::bind(&GlobalPlanner::mapCallBack, this,
                                                                                    std::placeholders::_1));

        targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(config.targetTopic, 1,
                                                                               std::bind(&GlobalPlanner::targetCallBack,
                                                                                         this, std::placeholders::_1));
    }

    inline void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!mapInitialized) {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *) (&msg->data[0]);
            for (size_t i = 0; i < total; i++) {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2])) {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
        }
    }

    inline void plan() {
#ifdef  STATIC_GO

        RCLCPP_WARN_STREAM(this->get_logger(), "goal point is " << startGoal[startGoal.size() - 1]);
#endif
        if (startGoal.size() == 2) {
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1) {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams)) {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol))) {
                    return;
                }

                if (traj.getPieceNum() > 0) {
                    trajStamp = this->get_clock()->now().seconds();
                    visualizer.visualize(traj, route);
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (mapInitialized) {
            if (startGoal.size() >= 2) {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                 (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0) {
                // For simplicity, start point position to (0,0,0)

#ifdef  STATIC_GO

                auto start = Eigen::Vector3d::Zero();
                goal = (Eigen::Vector3d() << 44.0, -11.4689, 0.551743).finished();
                visualizer.visualizeStartGoal(goal, 0.2, startGoal.size());
                startGoal.emplace_back(start);
                startGoal.emplace_back(goal);
#else
                auto start = Eigen::Vector3d::Zero();
                visualizer.visualizeStartGoal(goal, 0.2, startGoal.size());
                startGoal.emplace_back(start);
                startGoal.emplace_back(goal);
#endif
            } else {
                RCLCPP_WARN(this->get_logger(), "Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

    inline void process() {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;


        auto &tailsitter_data = tailsitter_df::Data::getInstance();
        auto &tailsitter_model = tailsitter_df::Model::getInstance();
        tailsitter_data.reset(tailsitter_model);
        if (traj.getPieceNum() > 0) {
            const double delta = this->get_clock()->now().seconds() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration()) {
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                autodiff::VectorXreal z;
                z.resize(9);
                z << traj.getVel(delta), traj.getAcc(delta), traj.getAcc(delta);
                tailsitter_df::df_forward(z);

                auto quat_df = tailsitter_data.getQuaternion();
                Eigen::Quaterniond q_b1_b2(0, 0.707, 0, 0.707);
                auto quat_q = quat_df * q_b1_b2;
                quat(0) = quat_q.w();
                quat(1) = quat_q.x();
                quat(2) = quat_q.y();
                quat(3) = quat_q.z();

#ifdef  DEBUG_VIZ
                double alpha = tailsitter_data.alpha.val() * 180 / M_PI;
                visualizer.visualizeFloat(alpha, "alpha");
                double thr = tailsitter_data.at.norm().val();
                double speed = traj.getVel(delta).norm();


                visualizer.visualizeFloat(speed, "speed");
                visualizer.visualizeFloat(thr, "at");
#endif
                visualizer.visualizeUAVTraj(traj.getPos(delta), quat);

                // visualizer.visualizeSphere(traj.getPos(delta),
                //                            config.dilateRadius);
            }
        } else {
            tailsitter_data.reset(tailsitter_df::Model::getInstance());

        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto global_planner = std::make_shared<GlobalPlanner>();


    rclcpp::Rate lr(1000);
    while (rclcpp::ok()) {
        global_planner->process();
        rclcpp::spin_some(global_planner);
        lr.sleep();
    }

    return 0;
}
