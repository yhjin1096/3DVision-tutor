#include "ceres/ceres.h"
#include "glog/logging.h"

class Pose2dErrorTerm
{
    public:
        // Pose2dErrorTerm(double measured[]) : p_measured(measured[0], measured[1]), theta_measured(measured[3]){}

        template <typename T>
        void operator()(const T* const a, const T* const b, T* residual) const{
            std::cout << "here" << std::endl;
        }


    private:
        const Eigen::Vector2d p_measured;
        double theta_measured;
};

int main(int argc, char** argv)
{
    const int *a, *b;
    int *c;
    
    Pose2dErrorTerm test;
    
    test(a, b, c);
    
    return 0;
}