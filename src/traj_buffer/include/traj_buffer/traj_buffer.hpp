#include <sys/types.h>

#include <Eigen/Dense>
#include <interface/msg/ref_trajectory.hpp>
#include <interface/msg/ref_trajectory_bullet.hpp>
#include <interface/msg/state.hpp>
#include <interface/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using TrajecotryPoint  = interface::msg::TrajectoryPoint;
using TrajecotryBullet = interface::msg::RefTrajectoryBullet;
using Trajectory       = interface::msg::RefTrajectory;
// constexpr double kUpdateIncrementTime = 0.02;
// constexpr int kRefStateDims = 9;
constexpr int    kRefTrajBulletSize  = TrajecotryBullet::SIZE;
constexpr int    kDownSamplingFactor = 5;
constexpr double kControlTime        = 0.1;
constexpr int    kRefPointDims       = TrajecotryPoint::POINT_DIM;
constexpr int    kYbStartIndex       = TrajecotryPoint::YB_START_INDEX;
constexpr double kThrustPerRotor     = 9.8 * 1.152 / 4;
class TrajectoryBufferCallbacks
{
   public:
    enum class RefTrajType
    {
        PLANNING,
        FIXED
    };
    RefTrajType traj_type_ = RefTrajType::FIXED;

   public:
    double                                                 last_v_norm_ = 0.0;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   ref_can_set_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   mode_trigger_sub_;
    rclcpp::Subscription<Trajectory>::SharedPtr            traj_sub_;
    rclcpp::Subscription<interface::msg::State>::SharedPtr state_sub_;

    rclcpp::Publisher<TrajecotryBullet>::SharedPtr traj_bullet_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                      ref_bullet_vis;
    const rclcpp::QoS reliabe_qos_keep_last =
        rclcpp::QoS(10)
            .reliable()
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            .keep_last(1);

   public:
    virtual void initSubsPubs()                                            = 0;
    virtual void stateCallback(const interface::msg::State::SharedPtr msg) = 0;
    virtual void refTrajectoryBulletPublish(
        std_msgs::msg::Bool::SharedPtr msg)                          = 0;
    virtual void updateTrajCallback(const Trajectory::SharedPtr msg) = 0;
    virtual void modeTriggerCallback(
        const std_msgs::msg::Bool::SharedPtr msg) = 0;
    virtual void declareParameters()              = 0;
    virtual void updateParameters()               = 0;
};

class TrajectoryBuffer : public rclcpp::Node, private TrajectoryBufferCallbacks
{
   private:
    size_t index_          = 0;
    bool   init_traj_set_  = false;
    bool   tracking_valid_ = false;
    double cz_predict_     = 0.0;
    // bool
    Trajectory       trajectory_;
    TrajecotryBullet msg_planning_traj_;
    TrajecotryBullet msg_fixed_point_;
    Eigen::Vector3d  yb_eigen_            = Eigen::Vector3d(1.0, 0.0, 0.0);
    double           traj_sampling_time_  = 0.02;
    double           traj_control_time_   = kControlTime;
    uint             down_sampling_factor = kDownSamplingFactor;
    bool             circled_             = false;

   private:
    void initSubsPubs() override;
    void declareParameters() override;
    void updateParameters() override;
    void stateCallback(const interface::msg::State::SharedPtr msg) override;
    void refTrajectoryBulletPublish(
        std_msgs::msg::Bool::SharedPtr msg) override;
    void modeTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg) override;
    void updateTrajCallback(const Trajectory::SharedPtr msg) override;

    inline bool checkTrajectoryTermination();

   public:
    TrajectoryBuffer();
};
