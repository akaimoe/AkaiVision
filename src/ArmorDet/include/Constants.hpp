#pragma once

#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"

// #define DEBUG      //打印相关信息
// #define Yolo       //是否启用Yolo检测装甲板ID
#ifdef Yolo
// #define CUDA    //是否启动CUDA加速（搭载Nidia显卡可开启，注意安装CUDA驱动）
#endif

constexpr double kDegreeToRadCoef = CV_PI / 180.0;    //角度制转弧度制
constexpr double kRadToDegreeCoef = 180.0 / CV_PI;    //弧度制转角度制

cv::Rect    fullROIRect;

struct AimbotConfig
{
    // 灯条相关参数
    float lightAspectRatioMax;
    float lightAspectRatioMin;
    float lightAreaMax;
    float lightAreaMin;
    float lightAngleMax;
    float lightMaxDiffRatioY;
    float lightMinDiffRatioX;
    float armorMaxAspectRatio;
    float armorMinAspectRatio;
    float lightAngleDiff;
    float lightLenDiff;
    float lightAreaDiffRatio;
    float armorLightAreaDiffRatio;
    float bigArmorRatio;
    int thresHoldColor;
    int thresHoldBrightness;
    std::string modlePath;
} aimbotConfig;

struct ImageLoader
{
    // 输入图片相关
    std::string imgPath;
    int frameWidth;
    int frameHeight;
    int myColor;
    int visionMode;
} imageConfig;

enum class ArmorType
{
    None,  // 0
    Big,
    Small,
    Windmill
};

enum class RMColor  //1 for blud, 2 for red
{
    None,
    Blue,  //1
    Red    //2
};

struct FrameInfo
{
    cv::Mat frame;
    ros::Time timestamp;
};

cv::Rect setROIRect(const cv::Rect& TargetArmorBBox, const double sizeX, const double sizeY)
{
    cv::Size newSize(TargetArmorBBox.width * sizeX, TargetArmorBBox.height * sizeY);
    double dx = fabs(sizeX - 1) / 2.0 * TargetArmorBBox.width;
    double dy = fabs(sizeY - 1) / 2.0 * TargetArmorBBox.height;
    double signX = (sizeX >= 1 ? 1 : -1);
    double signY = (sizeY >= 1 ? 1 : -1);
    cv::Point newTL(TargetArmorBBox.x - signX * dx, TargetArmorBBox.y - signY * dy);

    cv::Rect newRect = cv::Rect(newTL, newSize) & fullROIRect;
    if (newRect.area() == 0)
        newRect = fullROIRect;

    return newRect;
}


bool InitConfigs(const std::string& pathToConfig, const std::string& pathToCalibFile)
{
    cv::FileStorage fsReader(pathToConfig, cv::FileStorage::READ);
    if (!fsReader.isOpened())
    {
        return false;
    }

    cv::FileNode aimbotNode = fsReader["aimbot"];
    aimbotConfig.lightAspectRatioMax = (float)aimbotNode["LightAspectRatioMax"];
    aimbotConfig.lightAspectRatioMin = (float)aimbotNode["LightAspectRatioMin"];
    aimbotConfig.lightAreaMax = (float)aimbotNode["LightAreaMax"];
    aimbotConfig.lightAreaMin = (float)aimbotNode["LightAreaMin"];
    aimbotConfig.lightAngleMax = (float)aimbotNode["LightAngleMax"];
    aimbotConfig.lightMaxDiffRatioY = (float)aimbotNode["LightMaxDiffRatioY"];
    aimbotConfig.lightMinDiffRatioX = (float)aimbotNode["LightMinDiffRatioX"];
    aimbotConfig.armorMaxAspectRatio = (float)aimbotNode["ArmorMaxAspectRatio"];
    aimbotConfig.armorMinAspectRatio = (float)aimbotNode["ArmorMinAspectRatio"];
    aimbotConfig.lightAngleDiff = (float)aimbotNode["LightAngleDiff"];
    aimbotConfig.lightLenDiff = (float)aimbotNode["LightLenDiff"];
    aimbotConfig.lightAreaDiffRatio = (float)aimbotNode["LightAreaDiffRatio"];
    aimbotConfig.armorLightAreaDiffRatio = (float)aimbotNode["ArmorLightAreaDiffRatio"];
    aimbotConfig.bigArmorRatio = (float)aimbotNode["BigArmorRatio"];
    aimbotConfig.modlePath = (std::string)aimbotNode["ModlePath"];
    aimbotConfig.thresHoldColor = (int)aimbotNode["ThresHoldColor"];
    aimbotConfig.thresHoldBrightness = (int)aimbotNode["ThresHoldBrightness"];

    cv::FileNode imageNode = fsReader["imageLoader"];
    imageConfig.imgPath = (std::string)imageNode["ImgPath"];
    imageConfig.frameWidth = (int)imageNode["FrameWidth"];
    imageConfig.frameHeight = (int)imageNode["FrameHeight"];
    imageConfig.myColor = (int)imageNode["MyColor"];
    imageConfig.visionMode = (int)imageNode["VisionMode"];

    fsReader.release();

    fullROIRect = cv::Rect(0, 0, imageConfig.frameWidth, imageConfig.frameHeight);

    return true;
}
