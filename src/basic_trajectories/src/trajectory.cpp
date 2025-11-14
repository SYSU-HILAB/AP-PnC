#include <basic_trajectories/trajectory.hpp>

namespace basic_trajectories
{

    Trajectory::Trajectory()
    {
        type = Type::UNDEFINED;
    }
    // Trajectory

    Mat3x4 Trajectory::point(double t)
    {
        static Mat3x4 point;
        point.col(0) = pos(t);
        point.col(1) = vel(t);
        point.col(2) = acc(t);
        return point;
    }

    PoseArray& Trajectory::toMsg(int resolution)
    {
        static PoseArray msg;
        msg.poses.clear();
        double totaltime      = 2 * M_PI / angular_speed;
        double resolutionTime = totaltime / resolution;
        Vec3E  world_x        = Vec3E::UnitX();
        for (double t = 0; t <= totaltime + 1e-6; t += resolutionTime)
        {
            geometry_msgs::msg::Pose pose;
            auto                     p = pos(t);
            pose.position.x            = p[0];
            pose.position.y            = p[1];
            pose.position.z            = p[2];

            auto v = vel(t);
            v.normalize();
            double          dot_product  = v.dot(world_x);
            Eigen::Vector3d crossProduct = world_x.cross(v);
            double          yaw          = acos(dot_product);

            if (crossProduct.z() < 0)
            {
                yaw = 2 * M_PI - yaw;
            }
            pose.orientation.w = cos(yaw / 2);
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = sin(yaw / 2);
            msg.poses.push_back(pose);
            msg.header.frame_id = "world";
        }
        return msg;
    }

    std::vector<Mat3x4> Trajectory::samplingPointArray(int num)
    {
        std::vector<Mat3x4> res;
        double              totaltime      = 2 * M_PI / angular_speed;
        double              resolutionTime = totaltime / num;
        for (double t = 0; t <= totaltime + 1e-6; t += resolutionTime)
        {
            res.push_back(point(t));
        }
        return res;
    }
}  // namespace basic_trajectories
