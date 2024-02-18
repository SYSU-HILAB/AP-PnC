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

struct Config
{
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

    Config(rclcpp::Node *node)
    {
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
        chiVec = node->declare_parameter("ChiVec", std::vector<double>({1.0e+4, 1.0e+4, 1.0e+4, 1.0e+4, 1.0e+5}));
        smoothingEps = node->declare_parameter("SmoothingEps", 1.0);
        integralIntervs = node->declare_parameter("IntegralIntervs", 1);
        relCostTol = node->declare_parameter("RelCostTol", 1.0);
    }

    void UpdateConfig(const rclcpp::Node *node)
    {
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
    }
};

class GlobalPlanner : public rclcpp::Node
{
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
          visualizer(this)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        config.UpdateConfig(this);


        mapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(config.mapTopic, 1, std::bind(&GlobalPlanner::mapCallBack, this, std::placeholders::_1));

        targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(config.targetTopic, 1, std::bind(&GlobalPlanner::targetCallBack, this, std::placeholders::_1));
    }

    inline void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
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

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
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

            if (route.size() > 1)
            {
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
                                   physicalParams))
                {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    trajStamp = this->get_clock()->now().seconds();
                    visualizer.visualize(traj, route);
                }
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),"Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = this->get_clock()->now().seconds() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::msg::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub->publish(speedMsg);
                visualizer.thrPub->publish(thrMsg);
                visualizer.tiltPub->publish(tiltMsg);
                visualizer.bdrPub->publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto global_planner = std::make_shared<GlobalPlanner>();


    rclcpp::Rate lr(1000);
    while (rclcpp::ok())
    {
        global_planner->process();
        rclcpp::spin_some(global_planner);
        lr.sleep();
    }

    return 0;
}
