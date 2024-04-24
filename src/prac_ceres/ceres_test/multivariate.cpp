#include "ceres/ceres.h"
#include "glog/logging.h"

struct CostFunctor1 {
    template<typename T>
    bool operator()(const T *const a, const T *const b, T *residual) const {
        residual[0] = a[0] - b[0];
        residual[1] = a[0] - b[1];
        residual[2] = a[0] - b[2];

        residual[3] = a[1] - b[3];
        residual[4] = a[1] - b[4];
        residual[5] = a[1] - b[5];

        residual[6] = a[2] - b[6];
        residual[7] = a[2] - b[7];
        residual[8] = a[2] - b[8];
        return true;
    }
};

struct CostFunctor2 {
    template<typename T>
    bool operator()(const T *const a, const T *const b, T *residual) const {
        residual[0] = a[0] - b[0];
        return true;
    }
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    
    // double a[3] = {}; or double *a = (double *) malloc(3 * sizeof(double)); or Eigen::Vector3d a;
    std::vector<double> a, b;
    
    a.push_back(1.0);
    a.push_back(1.0);
    a.push_back(1.0);

    b.push_back(1.2);
    b.push_back(1.1);
    b.push_back(1.01);

    b.push_back(1.3);
    b.push_back(1.2);
    b.push_back(1.02);
    
    b.push_back(1.4);
    b.push_back(1.3);
    b.push_back(1.03);

    ceres::Problem problem;

    std::vector<double> initial_a = a, initial_b = b;

    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctor1, 9, 3, 9>(new CostFunctor1);
    problem.AddResidualBlock(cost_function, NULL, &a[0], &b[0]);

    
    // ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctor2, 1, 1, 1>(new CostFunctor2);

    // for(int i = 0; i < a.size(); i++)
    // {
    //     problem.AddResidualBlock(cost_function, NULL, &a[i], &b[i*3]);
    //     problem.AddResidualBlock(cost_function, NULL, &a[i], &b[i*3+1]);
    //     problem.AddResidualBlock(cost_function, NULL, &a[i], &b[i*3+2]);
    // }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "a_x: " << initial_a[0] << " -> " << a[0] << "\n";
    std::cout << "a_y: " << initial_a[1] << " -> " << a[1] << "\n";
    std::cout << "a_z: " << initial_a[2] << " -> " << a[2] << "\n";

    std::cout << "b_x: " << initial_b[0] << " -> " << b[0] << "\n";
    std::cout << "b_y: " << initial_b[1] << " -> " << b[1] << "\n";
    std::cout << "b_z: " << initial_b[2] << " -> " << b[2] << "\n";
    std::cout << "b_x: " << initial_b[3] << " -> " << b[3] << "\n";
    std::cout << "b_y: " << initial_b[4] << " -> " << b[4] << "\n";
    std::cout << "b_z: " << initial_b[5] << " -> " << b[5] << "\n";
    std::cout << "b_x: " << initial_b[6] << " -> " << b[6] << "\n";
    std::cout << "b_y: " << initial_b[7] << " -> " << b[7] << "\n";
    std::cout << "b_z: " << initial_b[8] << " -> " << b[8] << "\n";

    return 0;
}