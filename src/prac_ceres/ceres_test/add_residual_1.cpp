#include "ceres/ceres.h"
#include "glog/logging.h"

struct CostFunctor1 {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};
struct CostFunctor2 {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = 5.0 - x[0];
        return true;
    }
};
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    double x = 0.5;
    const double initial_x = x;

    ceres::Problem problem;
    ceres::CostFunction *cost_function1 = new ceres::AutoDiffCostFunction<CostFunctor1, 1, 1>(new CostFunctor1);
    ceres::CostFunction *cost_function2 = new ceres::AutoDiffCostFunction<CostFunctor2, 1, 1>(new CostFunctor2);

    problem.AddResidualBlock(cost_function1, NULL, &x);
    problem.AddResidualBlock(cost_function2, NULL, &x);

    ceres::Solver::Options options;
    options.eta = 1e-4;
    options.function_tolerance = 1e-10;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "x: " << initial_x << " -> " << x << std::endl;


    return 0;
}