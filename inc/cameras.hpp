#pragma once
#include <iostream>
#include <algorithm>
#include <chrono>
#include <string>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "hikcam.hpp"

#define CAMERAS_CHECK(state, str) \
    do { \
        if (!(state)) { \
            std::cerr << (str) << std::endl; \
            return -1; \
        } \
    } while (false)

class cameras{

public:
    enum MarkType {
        MARK_CROSS = 0x00,                      // 正十字
        MARK_X     = 0x01,                      // "X"
    };

public:
    //------------------------------初始化函数------------------------------
    cameras() : _intv{0}, _last_read(std::chrono::high_resolution_clock::now()), _idx(0){                                                         
        std::fill_n(this->_intv, sizeof(this->_intv) / sizeof(this->_intv[0]), 0x7f7f7f7f);
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_FATAL);                   // 设置opencv打印日志的等级, 不再打印warn信息
        this->_cam_type = "x";

        this->auto_detect_cam();
        this->print_cam_info(this->type_is_useful(this->_cam_type));
    }

    cameras(std::string cam_type) : _intv{0}, _last_read(std::chrono::high_resolution_clock::now()), _idx(0){
        std::fill_n(this->_intv, sizeof(this->_intv) / sizeof(this->_intv[0]), 0x7f7f7f7f);
        cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_FATAL);                   // 设置opencv打印日志的等级, 不再打印warn信息
        this->_cam_type = "x";

        if (!this->type_is_useful(cam_type)) {
            std::cerr << "param: " << cam_type << " is not useful" << std::endl;
            std::cerr << "automatically find available devices" << std::endl;
            auto_detect_cam();
        }
        else 
            this->_cam_type = cam_type;
        this->print_cam_info(this->type_is_useful(this->_cam_type));
    }

    //------------------------------打开摄像头------------------------------
    bool open() {
        if (this->_cam_type == "i") {
            this->_cam_industry = cv::VideoCapture(0);                  // TODO: 修改成可以处理多摄像头
            return this->_cam_industry.isOpened();
        }
        else if (this->_cam_type == "h") {
            return this->_cam_hik.open(0);                              // TODO: 修改成可以处理多摄像头
        }
        else if (this->_cam_type == "d") { 
            rs2::context ctx;
            auto list = ctx.query_devices(); 
            if (list.size() > 0)  {
                this->_cam_realsense_pipe = rs2::pipeline(ctx);
                return true;
            }
        }
        return false;
    }

    //------------------------------开始采集------------------------------
    bool start() {
        if (this->_cam_type == "i") {
            return true;
        }
        else if (this->_cam_type == "h") {
            return this->_cam_hik.start();
        }
        else if (this->_cam_type == "d") {
            this->_cam_realsense_pipe.start();                          // TODO: 无法判断是否开启成功
            return true;
        }
        return false;
    }

    //------------------------------停止采集------------------------------
    bool stop() {
        if (this->_cam_type == "i") {
            this->_cam_industry.release();
            return !this->_cam_industry.isOpened();
        }
        else if (this->_cam_type == "h") {
            return this->_cam_hik.stop();
        }
        else if (this->_cam_type == "d") {
            this->_cam_realsense_pipe.stop();
            return true;
        }
        return false;
    }

    //------------------------------获取一帧图片------------------------------
    cv::Mat get_frame() {
        if (this->_cam_type == "i") {
            this->_cam_industry.read(this->_frame);
        }
        else if (this->_cam_type == "h") {
            this->_frame = this->_cam_hik.read();
        }
        else if (this->_cam_type == "d") {
            this->_frame_realsense = this->_cam_realsense_pipe.wait_for_frames();
            rs2::frame color_frame = this->_frame_realsense.get_color_frame();

            const int w = color_frame.as<rs2::video_frame>().get_width();
            const int h = color_frame.as<rs2::video_frame>().get_height();

            cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::cvtColor(image, this->_frame, cv::COLOR_RGB2BGR);
        }
        else {
            this->_frame.release();
        }

        auto endtime = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(endtime - this->_last_read);
        this->_intv[this->_idx++] = us.count();
        this->_last_read = endtime;
        if (this->_idx == sizeof(this->_intv) / sizeof(this->_intv[0]))
            this->_idx = 0;
        return this->_frame;
    }


    //------------------------------获取深度图像指定坐标的数据------------------------------
    double get_depth(int x, int y) {
        if (this->_cam_type == "d") {
            rs2::depth_frame depth = this->_frame_realsense.get_depth_frame();
            return depth.get_distance(std::clamp(x, 0, depth.get_width()), std::clamp(y, 0, depth.get_height()));
        }
        return 0.0;
    }

    //------------------------------获取fps------------------------------
    double get_fps()
    {
        uint64_t total = 0;
        uint64_t size = 0;
        for (auto x : this->_intv)
        {
            if (x != 0x7f7f7f7f)
            {
                total += x;
                size++;
            }
        }
        return 1.0e6 / (total / double(size));
    }

    //------------------------------画图形------------------------------
    void draw_mark(int x, int y, MarkType mark_type = MARK_CROSS, cv::Scalar color = cv::Scalar(0, 0, 255), int line_length = 20, int line_width = 3) {
        const int frame_widht = this->_frame.cols;
        const int frame_height = this->_frame.rows;

        if (mark_type == MARK_CROSS) {
            cv::line(this->_frame, cv::Point(x, y - line_length), cv::Point(x, y + line_length), color, line_width);
            cv::line(this->_frame, cv::Point(x - line_length, y), cv::Point(x + line_length, y), color, line_width);
        }
        else if (mark_type == MARK_X) {
            cv::line(this->_frame, cv::Point(x - line_length, y - line_length), cv::Point(x + line_length, y + line_length), color, line_width);
            cv::line(this->_frame, cv::Point(x + line_length, y - line_length), cv::Point(x - line_length, y + line_length), color, line_width);
        }
    }

    std::string get_cam_type() { return this->_cam_type; }

private:
    bool type_is_useful(std::string cam_type) {
        if (cam_type == "h" || cam_type == "i" || cam_type == "d")
            return true;
        return false;
    }

    bool auto_detect_cam() {
        this->_cam_type = "x";
        do {
            if (this->cam_is_accessible_industry()) {
                this->_cam_type = "i";
                break;
            }
            
            else if (this->cam_is_accessible_hik()) {
                this->_cam_type = "h";
                break;
            }
            else if (this->cam_is_accessible_realsense()) {
                this->_cam_type = "d";
                break;
            }
        }while(0);
        return type_is_useful(this->_cam_type);
    }

    bool cam_is_accessible_industry() {
        //------------------------------检查工业摄像头是否可用------------------------------
        cv::VideoCapture cap(0);
        bool is_accessible = false;
        if (cap.isOpened()) {
            is_accessible = true;
            cap.release();
        }

        return is_accessible;
    }

    bool cam_is_accessible_hik() {
        //------------------------------检查海康摄像头是否可用------------------------------
        void* handle = NULL;
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        int nRet = MV_OK;
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            return false;
        }

        bool is_accessible = false;
        if (stDeviceList.nDeviceNum > 0) {
            do {
                // 尝试打开摄像头
                MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
                MV_CC_OpenDevice(handle);

                is_accessible = MV_CC_IsDeviceConnected(handle);

                // 关闭摄像头并删除句柄
                MV_CC_CloseDevice(handle);
                MV_CC_DestroyHandle(handle);
            }while(0);
        }

        return is_accessible;
    }

    bool cam_is_accessible_realsense() {
        //------------------------------检查深度摄像头是否可用------------------------------
        rs2::context ctx;
        rs2::device_list dev_list = ctx.query_devices(); 
        bool is_accessible = false;

        if (dev_list.size() > 0) {
            rs2::device dev = dev_list[0];                              // TODO: 修改为更加灵活的版本
            try {
                auto pipeline = rs2::pipeline(ctx);
                pipeline.start();                                       // start 失败的话就不用 stop了
                pipeline.stop();
                is_accessible = true;
            } catch (const rs2::error &e) {
                is_accessible = false;
            }
        }

        return is_accessible;
    }

    void print_cam_info(bool state) {
        if (state) {
            if (this->_cam_type == "i") {
                std::cout << "Now you are using industrial camera" << std::endl;
            }
            else if (this->_cam_type == "h") {
                std::cout << "Now you are using hik camera" << std::endl;
            }
            else if (this->_cam_type == "d") {
                std::cout << "Now you are using realsense camera" << std::endl;
            }
        }
        else {
            std::cout << "No cameras are available" << std::endl;
        }
    }

private:
    //------------------------------摄像头类型------------------------------
    hikcam _cam_hik;                                    // 海康摄像头
    cv::VideoCapture _cam_industry;                     // 工业摄像头
    rs2::pipeline _cam_realsense_pipe;                  // 深度摄像头的pipeline
    std::string _cam_type;                              // 摄像头的类型: h - hik摄像头; i - 工业摄像头; d - 深度摄像头

    //------------------------------数据帧------------------------------
    cv::Mat _frame;                                     // 图片处理后的opencv frame
    rs2::frameset _frame_realsense;                     // 深度相机视频帧


    //------------------------------计算fps------------------------------
    uint32_t _intv[10];                                 // 以微秒为单位的间隔
    uint32_t _idx;                                      
    decltype(std::chrono::high_resolution_clock::now()) _last_read; 

};
