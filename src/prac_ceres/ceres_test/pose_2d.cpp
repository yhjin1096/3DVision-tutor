#include "ceres/ceres.h"
#include "glog/logging.h"
#include <vector>
#include <Eigen/Dense>

class Pose2dErrorTerm
{
    public:
        Pose2dErrorTerm(double measured[]) : p_measured(measured[0], measured[1]), theta_measured(measured[3]){}

        template <typename T>
        bool operator()(const T* const a, const T* const b, T* residual) const{
            const Eigen::Matrix<T, 2, 1> p_a(a[0], a[1]);
            const Eigen::Matrix<T, 2, 1> p_b(b[0], b[1]);

            // 원점 기준 a의 회전 행렬
            Eigen::Matrix<T, 2, 2> R_a;
            const T cos = ceres::cos(a[2]);
            const T sin = ceres::sin(a[2]);
            
            // R_a(0,0) = cos; R_a(0,1) = -sin; R_a(1,0) = sin; R_a(1,1) = cos;
            R_a << cos, -sin,
                   sin, cos;

            // 2D pose error terms. posiont 및 angle에 대한 error.
            const Eigen::Matrix<T, 2, 1> p_diff = R_a.transpose() * (p_b - p_a) - p_measured.cast<T>(); // T로 형변환
            auto theta_diff =(b[2] - a[2]) - theta_measured;

            residual[0] = p_diff(0);
            residual[1] = p_diff(1);
            residual[2] = theta_diff;

            return true;
        }

    private:
        const Eigen::Vector2d p_measured;
        double theta_measured;
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    std::vector<double> a, b, ab_measured;

    a.push_back(3.0);
    a.push_back(1.0);
    a.push_back(0.523599);
    
    b.push_back(5.1);
    b.push_back(2.9);
    b.push_back(0.79);

    ab_measured.push_back(2.73205);
    ab_measured.push_back(0.73205);
    ab_measured.push_back(0.2618);

    const std::vector<double> initial_a = a, initial_b = b;

    ceres::Problem problem;

    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<Pose2dErrorTerm, 3, 3, 3>(new Pose2dErrorTerm(&ab_measured[0]));
    problem.AddResidualBlock(cost_function, NULL, &a[0], &b[0]);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "a(x, y, theta): (" <<initial_a[0]<<", "<<initial_a[1]<<", "<< initial_a[2] << ") ";
    std::cout << "--> (" <<a[0]<<", "<<a[1]<<", "<< a[2] << ")\n";
    
    std::cout << "b(x, y, theta): (" <<initial_b[0]<<", "<<initial_b[1]<<", "<< initial_b[2] << ") ";
    std::cout << "--> (" <<b[0]<<", "<<b[1]<<", "<< b[2] << ")\n";

    return 0;
}
