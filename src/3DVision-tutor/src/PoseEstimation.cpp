#include <3DVision-tutor/common.hpp>

class Node
{
    public:
        Camera left_cam, right_cam;

        Node()
        {
            right_cam.pose_eig.t << -right_cam.base_line, 0.0, 0.0;
            left_cam.pose_aff = Calculator::Eig_to_Aff(left_cam.pose_eig);
            right_cam.pose_aff = Calculator::Eig_to_Aff(right_cam.pose_eig);
        }

    private:
};

int main(int argc, char** argv)
{
    Node first_node;
    //second node pose 설정 후 pose estimation

    

    return 0;
}