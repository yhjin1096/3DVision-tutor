#include "prac_ceres/common.hpp"

//cost function 2개

struct CostFunctor1 {
    template<typename T>
    bool operator()(const T *const x, T *residual) const{
        residual[0] = 10.0 - x[0];
        return true;
    }
};

struct CostFunctor2 {
    template<typename T>
    bool operator()(const T *const x, T *residual) const{
        residual[0] = x[0];
        return true;
    }
};

struct CostFunctor3 {
    template<typename T>
    bool operator()(const T *const x, T *residual) const{
        residual[0] = 10.0 - x[0];
        residual[1] = x[0];
        return true;
    }
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    double x = 0.5;
    const double initial_x = x;
    
    ceres::Problem problem;

    //cost_function1,2 == cost_function3
    ceres::CostFunction *cost_function1 = new ceres::AutoDiffCostFunction<CostFunctor1, 1, 1>(new CostFunctor1);
    ceres::CostFunction *cost_function2 = new ceres::AutoDiffCostFunction<CostFunctor2, 1, 1>(new CostFunctor2);
    ceres::CostFunction *cost_function3 = new ceres::AutoDiffCostFunction<CostFunctor3, 2, 1>(new CostFunctor3);
    // problem.AddResidualBlock(cost_function1, NULL, &x);
    // problem.AddResidualBlock(cost_function2, NULL, &x);
    problem.AddResidualBlock(cost_function3, NULL, &x);

    ceres::Solver::Options options;
    //cost function의 변화량이 해당 값보다 작으면 iteration 중지
    options.function_tolerance = 1e-10;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;
    std::cout << "x: " << initial_x << " -> " << x << std::endl;

    return 0;
}