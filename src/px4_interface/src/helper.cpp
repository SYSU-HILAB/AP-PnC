#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
//     // Set up the solver

// ceres::Solve(options, &problem, &summary);

//     // Set up the solver
//     ceres::Solver::Options options;
//     options.linear_solver_type           = ceres::DENSE_QR;
//     options.minimizer_progress_to_stdout = true;
// ANSI escape codes for colors
#define RESET "\033[0m"     // Reset to default
#define BLACK "\033[30m"    // Black text
#define RED "\033[31m"      // Red text
#define GREEN "\033[32m"    // Green text
#define YELLOW "\033[33m"   // Yellow text
#define BLUE "\033[34m"     // Blue text
#define MAGENTA "\033[35m"  // Magenta text
#define CYAN "\033[36m"     // Cyan text
#define WHITE "\033[37m"    // White text

// Function to generate a random small perturbation quaternion
Eigen::Quaterniond random_perturbation(double angle_magnitude)
{
    // Random axis for rotation (normalized)
    std::random_device               rd;
    std::mt19937                     gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    Eigen::Vector3d axis(dis(gen), dis(gen), dis(gen));
    axis.normalize();  // Ensure it's a unit vector

    // Generate a small random angle for the perturbation
    std::uniform_real_distribution<> angle_dis(-angle_magnitude,
                                               angle_magnitude);
    double                           angle = angle_dis(gen);

    // Create a quaternion representing the small rotation
    Eigen::AngleAxisd angle_axis(angle, axis);
    return Eigen::Quaterniond(angle_axis);
}

// Define a struct for the cost function with automatic differentiation
struct YawCostFunctor
{
    static ceres::Solver::Options options;

    static ceres::Solver::Summary summary;
    static Eigen::Quaterniond     base_q_;  // The target quaternion

    YawCostFunctor(const Eigen::Quaterniond& q_i) : q_i_(q_i) {}

    static double constructProblemAndSolve(
        std::vector<Eigen::Quaterniond>& quaternions)
    {
        double         theta;
        ceres::Problem problem;

        // Add the yaw cost functor for each quaternion in the series
        for (size_t i = 0; i < quaternions.size(); ++i)
        {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<YawCostFunctor, 1, 1>(
                    new YawCostFunctor(quaternions[i]));
            problem.AddResidualBlock(cost_function, nullptr, &theta);
        }

        ceres::Solve(options, &problem, &summary);
        return theta;
    }

    template <typename T>
    bool operator()(const T* const theta, T* residual) const
    {
        // Create the yaw quaternion for the given theta
        T cos_half_theta = ceres::cos(theta[0] / T(2.0));
        T sin_half_theta = ceres::sin(theta[0] / T(2.0));

        Eigen::Quaternion<T> q_yaw(cos_half_theta, T(0.0), T(0.0),
                                   sin_half_theta);

        // Apply yaw quaternion to the target quaternion
        Eigen::Quaternion<T> transformed_q = q_yaw * base_q_.cast<T>();

        // Compute the difference between the transformed quaternion and q_i_
        Eigen::Quaternion<T> relative_q =
            transformed_q.inverse() * q_i_.cast<T>();
        // Compute the residual as the norm of the difference
        residual[0] =
            T(2.0) * ceres::atan2(relative_q.vec().norm(), relative_q.w());

        return true;
    }

    const Eigen::Quaterniond q_i_;  // The i-th quaternion in the series
};

ceres::Solver::Options YawCostFunctor::options = []()
{
    ceres::Solver::Options opt;
    // Customize 'options' if needed
    opt.max_num_iterations = 100;
    opt.linear_solver_type = ceres::DENSE_QR;
    return opt;
}();

Eigen::Quaterniond YawCostFunctor::base_q_ = Eigen::Quaterniond(1, 0, 0, 0);

ceres::Solver::Summary YawCostFunctor::summary;
// double theta   = YawCostFunctor::constructProblemAndSolve(quaternions);
// auto&  summary = YawCostFunctor::summary;
// std::cout << "Optimal yaw angle: " << theta << std::endl;

// int main()
// {
//     Eigen::Quaterniond base_quaternion(0, 1, 1, 0);

//     double             yaw = 0.5;
//     Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw,
//     Eigen::Vector3d::UnitZ()));

//     // base_quaternion = q_yaw * base_quaternion;
//     // Vector to hold quaternions with random perturbations
//     std::vector<Eigen::Quaterniond> quaternions;

//     // Generate 10 random perturbed quaternions
//     for (int i = 0; i < 10; ++i)
//     {
//         Eigen::Quaterniond perturbation =
//             random_perturbation(0.01);  // Small perturbation
//         Eigen::Quaterniond perturbed_q = perturbation * base_quaternion;
//         quaternions.push_back(perturbed_q);
//     }

//     // Define the target quaternion for the first problem: q(0, 1, 0, 0)
//     Eigen::Quaterniond target_q1(0, 1, 0, 0);
//     double theta   = YawCostFunctor::constructProblemAndSolve(quaternions);
//     auto&  summary = YawCostFunctor::summary;
//     // Output the results
//     std::cout << summary.FullReport() << std::endl;
//     // get the result of argument
//     std::cout << "Optimal yaw angle: " << theta << std::endl;

//     return 0;
// }
