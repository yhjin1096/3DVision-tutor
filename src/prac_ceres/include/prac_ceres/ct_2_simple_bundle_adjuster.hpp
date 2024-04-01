#ifndef ct_2_simple_bundle_adjuster_hpp
#define ct_2_simple_bundle_adjuster_hpp

#include "prac_ceres/common.hpp"

class BALProblem
{
    public:
        int num_cameras_;
        int num_points_;
        int num_observations_;
        int num_parameters_;
        
        std::vector<int> point_index_, camera_index_;
        std::vector<double> observations_, parameters_;
        
        bool LoadFile(const char* filename)
        {
            FILE* fptr = fopen(filename, "r");
            if (fptr == NULL) {
                return false;
            };

            FscanfOrDie(fptr, "%d", &num_cameras_); 
            FscanfOrDie(fptr, "%d", &num_points_); 
            FscanfOrDie(fptr, "%d", &num_observations_); 
            num_parameters_ = 9 * num_cameras_ + 3 * num_points_; 

            //problem-16-22106-pre.txt
            point_index_ = std::vector<int>(num_observations_);// 83718
            camera_index_ = std::vector<int>(num_observations_);// 83718
            observations_ = std::vector<double>(2 * num_observations_);// 83718*2 -> pixel_x, pixel_y
            parameters_ = std::vector<double>(num_parameters_);// 9*16 + 3*22106 // camera rotation(rodrigues 3 vector), tr_x, tr_y, tr_z, focal_length, k1, k2
            
            for (int i = 0; i < num_observations_; ++i) {
                FscanfOrDie(fptr, "%d", &camera_index_[i]);
                FscanfOrDie(fptr, "%d", &point_index_[i]);
                for (int j = 0; j < 2; ++j) {
                    FscanfOrDie(fptr, "%lf", &observations_ [2 * i + j]);
                }
            }

            for (int i = 0; i < num_parameters_; ++i)
                FscanfOrDie(fptr, "%lf", &parameters_[i]);
            
            return true;
        }

    private:
        template <typename T>
        void FscanfOrDie(FILE* fptr, const char* format, T* value) {
            int num_scanned = fscanf(fptr, format, value);
            if (num_scanned != 1) {
                LOG(FATAL) << "Invalid UW data file.";
            }
        }
};

struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x_(observed_x), observed_y_(observed_y){}

    template<typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];

        // y = R(angle_axis) * x;
        ceres::AngleAxisRotatePoint(camera, point, p);
        
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        //https://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html
        const T xp = -p[0]/p[2];
        const T yp = -p[1]/p[2];

        // Apply second and fourth order radial distortion.
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        const T r2 = xp * xp + yp * yp;
        const T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position. -> prediction의 pixel coordinate
        const T& focal = camera[6];
        const T predicted_x = focal * distortion * xp;
        const T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x_;
        residuals[1] = predicted_y - observed_y_;

        return true;
    }
    
    double observed_x_;
    double observed_y_;
};

#endif