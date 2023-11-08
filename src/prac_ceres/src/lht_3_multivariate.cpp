#include "prac_ceres/common.h"

struct CostFunctor {
    template<typename T>
    bool operator()(const T *const a, const T *const b, T *residual) const {
        residual[0] = a[0] - b[0];
        residual[1] = a[1] - b[1];
        residual[2] = a[2] - b[2];
        return true;
    }
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    std::vector<double> value1 = {1.0, 1.0, 1.0};
    std::vector<double> value2 = {1.2, 1.1, 1.01};
    const std::vector<double> init_value1 = {value1[0], value1[1], value1[2]};
    const std::vector<double> init_value2 = {value2[0], value2[1], value2[2]};

    ceres::Problem problem;

    // new ceres::AutoDiffCostFunction<CostFunctor, 1, 1> -> 1, 1은 각각 residual 수, 변수의 수를 의미
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 3, 3>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &value1[0], &value2[0]);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "value1[0]: " << init_value1[0] << " -> " << value1[0] << std::endl;
    std::cout << "value1[1]: " << init_value1[1] << " -> " << value1[1] << std::endl;
    std::cout << "value1[2]: " << init_value1[2] << " -> " << value1[2] << std::endl;

    std::cout << "value2[0]: " << init_value2[0] << " -> " << value2[0] << std::endl;
    std::cout << "value2[1]: " << init_value2[1] << " -> " << value2[1] << std::endl;
    std::cout << "value2[2]: " << init_value2[2] << " -> " << value2[2] << std::endl;

    return 0;
}