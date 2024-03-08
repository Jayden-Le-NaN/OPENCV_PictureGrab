#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "cameras.hpp"
int main() {
    //------------------------------init camera------------------------------
    cv::Mat frame;    
    cv::namedWindow("Cam", cv::WINDOW_NORMAL);
    cameras cam;
    CAMERAS_CHECK(cam.open(), "open camera fail");
    CAMERAS_CHECK(cam.start(), "start camera fail");

    int img_cnt = 0;
    std::string img_name;
    if (cam.get_cam_type() == "i") 
        img_name = "./img/industry_img/img_";
    else if (cam.get_cam_type() == "h") 
        img_name = "./img/hik_img/img_";
    else if (cam.get_cam_type() == "d") 
        img_name = "./img/realsense_img/img_";

    while(true) {
        frame = cam.get_frame();
        if (frame.empty())
            continue;
        

        cv::imshow("Cam", frame);
        char key = cv::waitKey(1);
        if (key == ' ') {
            char ch = cv::waitKey();
            if (ch == 'y') {
                cv::imwrite(img_name + std::to_string(img_cnt) + ".jpeg", frame);
                ++img_cnt;
                std::cout << "write img success" << std::endl;
            }
        }

        if (key == 27)
            break;
    }
    cam.stop();
    cv::destroyAllWindows();
    return 0;
}
