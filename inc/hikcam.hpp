#pragma once
#include "MvCameraControl.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include <vector>


class hikcam_info
{
public:
    uint16_t major_ver;
    uint16_t minor_ver;
    uint64_t mac_addr;
    uint32_t tlayer_type;
    std::string model_name;
    std::string user_defined_name;
};

class hikcam
{
public:
    hikcam() : handle(nullptr), bayer(1024, 1280, CV_8UC1), img(1024, 1280, CV_8UC3), empty(), intv{0}, last_read(std::chrono::high_resolution_clock::now()), idx(0)
    {
        std::fill_n(intv, sizeof(intv) / sizeof(intv[0]), 0x7f7f7f7f);
    }

    std::vector<hikcam_info> enumerate()
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        // enum device
        int nRet = MV_OK;
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            return {};
        }
        std::vector<hikcam_info> ret;
        for (int i = 0; i < stDeviceList.nDeviceNum; ++i)
        {
            ret.push_back(hikcam_info{.major_ver = stDeviceList.pDeviceInfo[i]->nMajorVer,
                                      .minor_ver = stDeviceList.pDeviceInfo[i]->nMinorVer,
                                      .mac_addr = ((uint64_t)stDeviceList.pDeviceInfo[i]->nMacAddrHigh << 32) | stDeviceList.pDeviceInfo[i]->nMacAddrLow,
                                      .tlayer_type = stDeviceList.pDeviceInfo[i]->nTLayerType,
                                      .model_name = (char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chModelName,
                                      .user_defined_name = (char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chUserDefinedName});
        }
        return ret;
    }

    bool open(int n)
    {
        int nRet = MV_OK;
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            return false;
        }
        if (n >= stDeviceList.nDeviceNum)
        {
            printf("parameter exceeds maximum of %d\n", stDeviceList.nDeviceNum);
            return false;
        }
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[n]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            return false;
        }
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    bool start()
    {
        int nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    const cv::Mat &read()
    {
        int nRet;
        MV_FRAME_OUT frame;
        nRet = MV_CC_GetImageBuffer(handle, &frame, ~0u);
        // nRet = MV_CC_Display(handle, (void *)w);
        if (MV_OK != nRet)
        {
            printf("MV_CC_GetImageBuffer fail! nRet [%x]\n", nRet);
            return empty;
        }
        memcpy(bayer.data, frame.pBufAddr, frame.stFrameInfo.nFrameLen);
        assert(frame.stFrameInfo.nWidth == 1280 && frame.stFrameInfo.nHeight == 1024);
        // printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d], nFrameLen[%d], enPixelType[%s]\n",
        //        frame.stFrameInfo.nWidth, frame.stFrameInfo.nHeight, frame.stFrameInfo.nFrameNum, frame.stFrameInfo.nFrameLen, format_name(frame.stFrameInfo.enPixelType));
        cv::cvtColor(bayer, img, cv::COLOR_BayerRG2RGB);
        // nRet = MV_CC_ConvertPixelType(handle, &conv);
        // sdk function is slow (~60ms per convertion)
        // use OpenCV instead (effective ~2ms)
        nRet = MV_CC_FreeImageBuffer(handle, &frame);
        if (nRet != MV_OK)
        {
            printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            return empty;
        }
        auto endtime = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(endtime - last_read);
        intv[idx++] = us.count();
        last_read = endtime;
        if (idx == sizeof(intv) / sizeof(intv[0]))
        {
            idx = 0;
        }
        return img;
    }
    bool stop()
    {
        int nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        // close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            return false;
        }
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            return false;
        }
        handle = nullptr;
        return true;
    }
    double fps()
    {
        uint64_t total = 0, size = 0;
        for (auto x : intv)
        {
            if (x != 0x7f7f7f7f)
            {
                total += x;
                size++;
            }
        }
        return 1.0e6 / (total / double(size));
    }
private:
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("The Pointer of pstMVDevInfo is NULL!\n");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
            printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
            printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
            printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }

        return true;
    }

    const char *format_name(enum MvGvspPixelType type)
    {
#define CHECK(x)   \
        if (type == x) \
            return #x;
        CHECK(PixelType_Gvsp_Undefined);
        CHECK(PixelType_Gvsp_Mono1p);
        CHECK(PixelType_Gvsp_Mono2p);
        CHECK(PixelType_Gvsp_Mono4p);
        CHECK(PixelType_Gvsp_Mono8);
        CHECK(PixelType_Gvsp_Mono8_Signed);
        CHECK(PixelType_Gvsp_Mono10);
        CHECK(PixelType_Gvsp_Mono10_Packed);
        CHECK(PixelType_Gvsp_Mono12);
        CHECK(PixelType_Gvsp_Mono12_Packed);
        CHECK(PixelType_Gvsp_Mono14);
        CHECK(PixelType_Gvsp_Mono16);
        CHECK(PixelType_Gvsp_BayerGR8);
        CHECK(PixelType_Gvsp_BayerRG8);
        CHECK(PixelType_Gvsp_BayerGB8);
        CHECK(PixelType_Gvsp_BayerBG8);
        CHECK(PixelType_Gvsp_BayerGR10);
        CHECK(PixelType_Gvsp_BayerRG10);
        CHECK(PixelType_Gvsp_BayerGB10);
        CHECK(PixelType_Gvsp_BayerBG10);
        CHECK(PixelType_Gvsp_BayerGR12);
        CHECK(PixelType_Gvsp_BayerRG12);
        CHECK(PixelType_Gvsp_BayerGB12);
        CHECK(PixelType_Gvsp_BayerBG12);
        CHECK(PixelType_Gvsp_BayerGR10_Packed);
        CHECK(PixelType_Gvsp_BayerRG10_Packed);
        CHECK(PixelType_Gvsp_BayerGB10_Packed);
        CHECK(PixelType_Gvsp_BayerBG10_Packed);
        CHECK(PixelType_Gvsp_BayerGR12_Packed);
        CHECK(PixelType_Gvsp_BayerRG12_Packed);
        CHECK(PixelType_Gvsp_BayerGB12_Packed);
        CHECK(PixelType_Gvsp_BayerBG12_Packed);
        CHECK(PixelType_Gvsp_BayerGR16);
        CHECK(PixelType_Gvsp_BayerRG16);
        CHECK(PixelType_Gvsp_BayerGB16);
        CHECK(PixelType_Gvsp_BayerBG16);
        CHECK(PixelType_Gvsp_RGB8_Packed);
        CHECK(PixelType_Gvsp_BGR8_Packed);
        CHECK(PixelType_Gvsp_RGBA8_Packed);
        CHECK(PixelType_Gvsp_BGRA8_Packed);
        CHECK(PixelType_Gvsp_RGB10_Packed);
        CHECK(PixelType_Gvsp_BGR10_Packed);
        CHECK(PixelType_Gvsp_RGB12_Packed);
        CHECK(PixelType_Gvsp_BGR12_Packed);
        CHECK(PixelType_Gvsp_RGB16_Packed);
        CHECK(PixelType_Gvsp_BGR16_Packed);
        CHECK(PixelType_Gvsp_RGBA16_Packed);
        CHECK(PixelType_Gvsp_BGRA16_Packed);
        CHECK(PixelType_Gvsp_RGB10V1_Packed);
        CHECK(PixelType_Gvsp_RGB10V2_Packed);
        CHECK(PixelType_Gvsp_RGB12V1_Packed);
        CHECK(PixelType_Gvsp_RGB565_Packed);
        CHECK(PixelType_Gvsp_BGR565_Packed);
        CHECK(PixelType_Gvsp_YUV411_Packed);
        CHECK(PixelType_Gvsp_YUV422_Packed);
        CHECK(PixelType_Gvsp_YUV422_YUYV_Packed);
        CHECK(PixelType_Gvsp_YUV444_Packed);
        CHECK(PixelType_Gvsp_YCBCR8_CBYCR);
        CHECK(PixelType_Gvsp_YCBCR422_8);
        CHECK(PixelType_Gvsp_YCBCR422_8_CBYCRY);
        CHECK(PixelType_Gvsp_YCBCR411_8_CBYYCRYY);
        CHECK(PixelType_Gvsp_YCBCR601_8_CBYCR);
        CHECK(PixelType_Gvsp_YCBCR601_422_8);
        CHECK(PixelType_Gvsp_YCBCR601_422_8_CBYCRY);
        CHECK(PixelType_Gvsp_YCBCR601_411_8_CBYYCRYY);
        CHECK(PixelType_Gvsp_YCBCR709_8_CBYCR);
        CHECK(PixelType_Gvsp_YCBCR709_422_8);
        CHECK(PixelType_Gvsp_YCBCR709_422_8_CBYCRY);
        CHECK(PixelType_Gvsp_YCBCR709_411_8_CBYYCRYY);
        CHECK(PixelType_Gvsp_RGB8_Planar);
        CHECK(PixelType_Gvsp_RGB10_Planar);
        CHECK(PixelType_Gvsp_RGB12_Planar);
        CHECK(PixelType_Gvsp_RGB16_Planar);
        CHECK(PixelType_Gvsp_Jpeg);
        CHECK(PixelType_Gvsp_Coord3D_ABC32f);
        CHECK(PixelType_Gvsp_Coord3D_ABC32f_Planar);
        CHECK(PixelType_Gvsp_Coord3D_AC32f);
        CHECK(PixelType_Gvsp_COORD3D_DEPTH_PLUS_MASK);
        CHECK(PixelType_Gvsp_Coord3D_ABC32);
        CHECK(PixelType_Gvsp_Coord3D_AB32f);
        CHECK(PixelType_Gvsp_Coord3D_AB32);
        CHECK(PixelType_Gvsp_Coord3D_AC32f_64);
        CHECK(PixelType_Gvsp_Coord3D_AC32f_Planar);
        CHECK(PixelType_Gvsp_Coord3D_AC32);
        CHECK(PixelType_Gvsp_Coord3D_A32f);
        CHECK(PixelType_Gvsp_Coord3D_A32);
        CHECK(PixelType_Gvsp_Coord3D_C32f);
        CHECK(PixelType_Gvsp_Coord3D_C32);
        CHECK(PixelType_Gvsp_Coord3D_ABC16);
        CHECK(PixelType_Gvsp_Coord3D_C16);
        CHECK(PixelType_Gvsp_HB_Mono8);
        CHECK(PixelType_Gvsp_HB_Mono10);
        CHECK(PixelType_Gvsp_HB_Mono10_Packed);
        CHECK(PixelType_Gvsp_HB_Mono12);
        CHECK(PixelType_Gvsp_HB_Mono12_Packed);
        CHECK(PixelType_Gvsp_HB_Mono16);
        CHECK(PixelType_Gvsp_HB_BayerGR8);
        CHECK(PixelType_Gvsp_HB_BayerRG8);
        CHECK(PixelType_Gvsp_HB_BayerGB8);
        CHECK(PixelType_Gvsp_HB_BayerBG8);
        CHECK(PixelType_Gvsp_HB_BayerRBGG8);
        CHECK(PixelType_Gvsp_HB_BayerGR10);
        CHECK(PixelType_Gvsp_HB_BayerRG10);
        CHECK(PixelType_Gvsp_HB_BayerGB10);
        CHECK(PixelType_Gvsp_HB_BayerBG10);
        CHECK(PixelType_Gvsp_HB_BayerGR12);
        CHECK(PixelType_Gvsp_HB_BayerRG12);
        CHECK(PixelType_Gvsp_HB_BayerGB12);
        CHECK(PixelType_Gvsp_HB_BayerBG12);
        CHECK(PixelType_Gvsp_HB_BayerGR10_Packed);
        CHECK(PixelType_Gvsp_HB_BayerRG10_Packed);
        CHECK(PixelType_Gvsp_HB_BayerGB10_Packed);
        CHECK(PixelType_Gvsp_HB_BayerBG10_Packed);
        CHECK(PixelType_Gvsp_HB_BayerGR12_Packed);
        CHECK(PixelType_Gvsp_HB_BayerRG12_Packed);
        CHECK(PixelType_Gvsp_HB_BayerGB12_Packed);
        CHECK(PixelType_Gvsp_HB_BayerBG12_Packed);
        CHECK(PixelType_Gvsp_HB_YUV422_Packed);
        CHECK(PixelType_Gvsp_HB_YUV422_YUYV_Packed);
        CHECK(PixelType_Gvsp_HB_RGB8_Packed);
        CHECK(PixelType_Gvsp_HB_BGR8_Packed);
        CHECK(PixelType_Gvsp_HB_RGBA8_Packed);
        CHECK(PixelType_Gvsp_HB_BGRA8_Packed);
        CHECK(PixelType_Gvsp_HB_RGB16_Packed);
        CHECK(PixelType_Gvsp_HB_BGR16_Packed);
        CHECK(PixelType_Gvsp_HB_RGBA16_Packed);
        CHECK(PixelType_Gvsp_HB_BGRA16_Packed);
        return "nothing";
    }

private:
    void *handle;
    cv::Mat bayer, img, empty;
    uint32_t intv[10]; // interval in microsecs
    uint32_t idx;
    decltype(std::chrono::high_resolution_clock::now()) last_read;
};
