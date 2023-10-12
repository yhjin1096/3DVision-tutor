#include <iostream>
#include <opencv2/viz.hpp>

int main( int /*argc*/, char** /*argv*/ )
{
    //창 만들기
    cv::viz::Viz3d myWindow("Viz Demo");
    //이벤트 루프 실행 -> e, E, q, Q 누르면 종료
    myWindow.spin();

    //myWindow와 완전 같은 창 만들기
    cv::viz::Viz3d sameWindow = cv::viz::getWindowByName("Viz Demo");
    sameWindow.spin();

    while(!sameWindow.wasStopped())
    {
        sameWindow.spinOnce(1, true);
    }
    return 0;
}