#include "prac_ceres/ct_2_simple_bundle_adjuster.hpp"


int main(int argc, char** argv)
{
    BALProblem bal_problem;
    bal_problem.LoadFile("/home/cona/Downloads/data");

    ceres::Problem problem;
    for(int i = 1; i < bal_problem.num_observations_; i++)
    {
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                                                new SnavelyReprojectionError(bal_problem.observations_[2*i], //x
                                                                            bal_problem.observations_[2*i+1])); //y
        problem.AddResidualBlock(cost_function, 
                                 NULL,
                                 &bal_problem.parameters_[bal_problem.camera_index_[i]*9],
                                 &bal_problem.parameters_[bal_problem.point_index_[i]*3 + bal_problem.num_cameras_*9]);
        
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    
    return 0;
}