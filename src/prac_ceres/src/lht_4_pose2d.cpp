#include "prac_ceres/common.h"

// 해당 cost function은 ceres example의 pose_graph_2d에서 볼 수 있음
class Pose2dErrorTerm{
    public:
    Pose2dErrorTerm(std::vector<double> measured)
    :p_measured(measured[0], measured[1]), theta_measured(measured[2]) {}

    template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const {
        
        const Eigen::Matrix<T, 2, 1> p_a(a[0], a[1]);
        const Eigen::Matrix<T, 2, 1> p_b(b[0], b[1]);

        Eigen::Matrix<T, 2, 2> R_a;
        R_a << ceres::cos(a[2]), -ceres::sin(a[2]),
               ceres::sin(a[2]), ceres::cos(a[2]);
        
        const Eigen::Matrix<T, 2, 1> p_diff = R_a.transpose() * (p_b - p_a) - p_measured.cast<T>();

        auto theta_diff = (b[2] - a[2]) - theta_measured;
        
        residual[0] = p_diff(0);
        residual[1] = p_diff(1);
        residual[2] = theta_diff;

        return true;
    }

    const Eigen::Vector2d p_measured;
    double theta_measured;
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    //x, y, radian
    std::vector<double> pose1 = {3.0, 1.0, 0.523599};
    std::vector<double> pose2 = {5.1, 2.9, 0.79};
    std::vector<double> relative_pose = {2.73205, 0.73205, 0.2618};
    const std::vector<double> init_pose1 = {pose1[0], pose1[1], pose1[2]};
    const std::vector<double> init_pose2 = {pose2[0], pose2[1], pose2[2]};
    
    ceres::Problem problem;

    // new ceres::AutoDiffCostFunction<CostFunctor, 1, 1> -> 1, 1은 각각 residual 수, 변수의 수를 의미
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<Pose2dErrorTerm, 3, 3, 3>(new Pose2dErrorTerm(relative_pose));
    problem.AddResidualBlock(cost_function, NULL, &pose1[0], &pose2[0]);
    // pose1 고정
    problem.SetParameterBlockConstant(&pose1[0]);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "pose1: " << init_pose1[0] << "," << init_pose1[1] << "," << init_pose1[2] << " -> " << pose1[0] << "," << pose1[1] << "," << pose1[2] << std::endl;
    std::cout << "pose2: " << init_pose2[0] << "," << init_pose2[1] << "," << init_pose2[2] << " -> " << pose2[0] << "," << pose2[1] << "," << pose2[2] << std::endl;
    
    return 0;
}