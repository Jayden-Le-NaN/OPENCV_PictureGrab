#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "cameras.hpp"
#include <yaml-cpp/yaml.h>
#include <cassert>
#include <fstream>

const std::string config_path = "/home/bupt-rc/Code/opencv/2024/030401_DataSetCollection/config.yaml";

int main() {
    //------------------------------init camera------------------------------
    cv::Mat frame;    
    cv::namedWindow("Cam", cv::WINDOW_NORMAL);
    cameras cam;
    CAMERAS_CHECK(cam.open(), "open camera fail");
    CAMERAS_CHECK(cam.start(), "start camera fail");
    //------------------------------load configuration------------------------------
    int img_cnt = 0;
    std::string img_name;
    YAML::Node config = YAML::LoadFile(config_path);
    assert(!config.IsNull() && "Invalid yaml file");

    if (cam.get_cam_type() == "i"){                     // industry camera
        assert(config["image_path"]["industry_img_path"].IsDefined());
        assert(config["image_cnt"]["industry_img_cnt"].IsDefined());
        img_name = config["image_path"]["industry_img_path"].as<std::string>() + "/img_";
        img_cnt = config["image_cnt"]["industry_img_cnt"].as<int>();
    }                      
    else if (cam.get_cam_type() == "h"){                // hik camera
        assert(config["image_path"]["hik_img_path"].IsDefined());
        assert(config["image_cnt"]["hik_img_cnt"].IsDefined());
        img_name = config["image_path"]["hik_img_path"].as<std::string>() + "/img_";
        img_cnt = config["image_cnt"]["hik_img_cnt"].as<int>();
    }
    else if (cam.get_cam_type() == "d"){                // realsense camera
        assert(config["image_path"]["realsense_img_path"].IsDefined());
        assert(config["image_cnt"]["realsense_img_cnt"].IsDefined());
        img_name = config["image_path"]["realsense_img_path"].as<std::string>() + "/img_";
        img_cnt = config["image_cnt"]["realsense_img_cnt"].as<int>();
    }

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

    //------------------------------refresh configuration------------------------------
    std::ofstream fout(config_path);
    if (cam.get_cam_type() == "i"){                     // industry camera
        config["image_cnt"]["industry_img_cnt"] = img_cnt;
    }                      
    else if (cam.get_cam_type() == "h"){                // hik camera
        config["image_cnt"]["hik_img_cnt"] = img_cnt;
    }
    else if (cam.get_cam_type() == "d"){                // realsense camera
        config["image_cnt"]["realsense_img_cnt"] = img_cnt;
    }
    fout << config;
    fout.close();

    return 0;
}
