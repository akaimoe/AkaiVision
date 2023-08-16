//Constants for PoseSolver

#pragma once

#include<opencv2/opencv.hpp>
#include<bits/stdc++.h>

std::string robotName;

template<typename T>
bool IsLegalNum(T x)
{
    return (!std::isnan(x) && !std::isinf(x));
}

class RMColor
{
public:
    RMColor() = default;
    RMColor(int x) {
        switch (x)
        {
        case 1:
            m_Color = Color::Red;
            break;
        case 2:
            m_Color = Color::Blue;
            break;
        case 3:
            m_Color = Color::Green;
            break;
        case 4:
            m_Color = Color::Yellow;
            break;
        default:
            m_Color = Color::Unknown;
            break;
        }
    }
    bool operator == (const RMColor& rhs)
    {
        return m_Color == rhs.m_Color;
    }
    RMColor& operator = (const RMColor& rhs)
    {
        this->m_Color = rhs.m_Color;
        return *this;
    }
    int CVChannelIdx()
    {
        int idx = 1;
        switch (m_Color)
        {
        case Color::Red:
            idx = 2;
            break;
        case Color::Blue:
            idx = 0;
            break;
        default:
            idx = 1;
            break;
        }

        return idx;
    }
    int Int() {
        return static_cast<int>(m_Color);
    }
    cv::Scalar CVScalar() {
        switch (m_Color)
        {
        case Color::Red:
            return cv::Scalar(0, 0, 255);
        case Color::Blue:
            return cv::Scalar(255, 0, 0);
        case Color::Green:
            return cv::Scalar(0, 255, 0);
        case Color::Yellow:
            return cv::Scalar(0, 255, 255);
        default:
            return cv::Scalar(62, 255, 192);
        }
    }
private:
    enum class Color
    {
        Unknown, Red, Blue, Green, Yellow
    };

    Color m_Color;
};

RMColor kColorUnknown(0), kColorRed(1), kColorBlue(2), kColorGreen(3), kColorYellow(4);


enum class VisionMode
{
    Disable = 0,
    Aimbot,
    SmallBuff,
    BigBuff
};


struct AimbotConfig {
    int thresBrightness;  // 根据环境光照调整这个阈值。摄像头曝光最好尽可能低；若有光晕，此阈值可调大一些
    int thresColor;       // 红蓝通道相减，判断是否为蓝/红色的阈值

    // 灯条过滤参数
    double lightBarMinArea;
    double lightBarMaxArea;
    double lightBarContourMinSolidity;
    double lightBarEllipseMinAspectRatio;  // 长轴:短轴
    double lightBarEllipseMaxAspectRatio;

    // 灯条对过滤参数
    double armorMaxAngleDiff;
    double armorMaxHeightDiffRatio;
    double armorMaxYDiffRatio;
    double armorMinXDiffRatio;

    // 装甲板过滤参数
    double armorBigArmorRatio;
    double armorSmallArmorRatio;
    double armorMinAspectRatio;
    double armorMaxAspectRatio;

    // 分数计算放缩系数
    double areaNormalizedBase;
    double sightOffsetNormalizedBase;

    // 漏检持续开火帧数
    int continousFireCount;

    // 大小装甲板开火偏角阈值
    double fireBiasYawSmall;
    double fireBiasPitchSmall;
    double fireBiasYawBig;
    double fireBiasPitchBig;

} aimbotConfig;


struct WindmillConfig {
    int thresBrightness;  // 根据环境光照调整这个阈值。摄像头曝光最好尽可能低；若有光晕，此阈值可调大一些
    int thresColor;       // 红蓝通道相减，判断是否为蓝/红色的阈值

    double armorMinArea;
    double armorMaxArea;
    double armorMinAspectRatio;
    double armorMaxAspectRatio;

    double centerMinArea;
    double centerMaxArea;

    double minRadius;
    double maxRadius;

    // 大小符的预测延时
    double timeDelaySmallBuff;
    double timeDelayBigBuff;

    // 风车装甲板开火偏角阈值
    double fireBiasYaw;
    double fireBiasPitch;
} windmillConfig;


struct ImageLoaderConfig {
    std::string pathToImageSrc; // 海康相机，图片文件夹，视频
    int frameWidth;
    int frameHeight;
    cv::Mat cameraMatrix, distCoeffs;
    RMColor myColor; // 0未知，1红，2蓝
    VisionMode visionMode;
} imageLoaderConfig;

enum class ArmorType
{
    None, Big, Small, Windmill
};

// 大小装甲板四个顶点的三维坐标
// 以装甲板中心为原点，向右为x正方向，向下为y正方向
// 坐标单位 mm
const double WIDTH_OF_ARMOR_SMALL = 133.0; // mm
const double HEIGHT_OF_ARMOR_SMALL = 56.0;
const std::vector<cv::Point3d> POINT_3D_OF_ARMOR_SMALL =
{
    cv::Point3d(-WIDTH_OF_ARMOR_SMALL / 2.0, -HEIGHT_OF_ARMOR_SMALL / 2.0, 0),	//tl
    cv::Point3d(WIDTH_OF_ARMOR_SMALL  / 2.0, -HEIGHT_OF_ARMOR_SMALL / 2.0, 0),	//tr
    cv::Point3d(WIDTH_OF_ARMOR_SMALL  / 2.0,  HEIGHT_OF_ARMOR_SMALL / 2.0, 0),	//br
    cv::Point3d(-WIDTH_OF_ARMOR_SMALL / 2.0,  HEIGHT_OF_ARMOR_SMALL / 2.0, 0),	//bl
    //cv::Point3d(0, 0, 0)    // center
};

const double WIDTH_OF_ARMOR_BIG = 235.0; // mm
const double HEIGHT_OF_ARMOR_BIG = 56.0;
const std::vector<cv::Point3d> POINT_3D_OF_ARMOR_BIG =
{
    cv::Point3d(-WIDTH_OF_ARMOR_BIG / 2.0, -HEIGHT_OF_ARMOR_BIG / 2.0, 0),	        //tl
    cv::Point3d(WIDTH_OF_ARMOR_BIG  / 2.0, -HEIGHT_OF_ARMOR_BIG / 2.0, 0),	        //tr
    cv::Point3d(WIDTH_OF_ARMOR_BIG  / 2.0,  HEIGHT_OF_ARMOR_BIG / 2.0, 0),	        //br
    cv::Point3d(-WIDTH_OF_ARMOR_BIG / 2.0,  HEIGHT_OF_ARMOR_BIG / 2.0, 0),		    //bl
    //cv::Point3d(0, 0, 0)    // center
};


//TODO 风车尺寸未添加
const double WIDTH_OF_ARMOR_WINDMILL = 235.0; // mm
const double HEIGHT_OF_ARMOR_WINDMILL = 56.0;
const std::vector<cv::Point3d> POINT_3D_OF_ARMOR_WINDMILL=
{
    cv::Point3d(-WIDTH_OF_ARMOR_WINDMILL / 2.0, -HEIGHT_OF_ARMOR_WINDMILL / 2.0, 0),	        //tl
    cv::Point3d(WIDTH_OF_ARMOR_WINDMILL  / 2.0, -HEIGHT_OF_ARMOR_WINDMILL / 2.0, 0),	        //tr
    cv::Point3d(WIDTH_OF_ARMOR_WINDMILL  / 2.0,  HEIGHT_OF_ARMOR_WINDMILL / 2.0, 0),	        //br
    cv::Point3d(-WIDTH_OF_ARMOR_WINDMILL / 2.0,  HEIGHT_OF_ARMOR_WINDMILL / 2.0, 0),		    //bl
    //cv::Point3d(0, 0, 0)    // center
};

constexpr double kDegreeToRadCoef = CV_PI / 180.0;    //角度制转弧度制
constexpr double kRadToDegreeCoef = 180.0 / CV_PI;    //弧度制转角度制


struct PoseSolverConfig
{
    // mm
    // double cameraToIMUX;
    // double cameraToIMUY;
    // double cameraToIMUZ;
    cv::Mat ArmorCS = (cv::Mat_<double>(1, 3) <<            //装甲板坐标系，默认以装甲板中心为目标点
                       0, 0, 0);

    cv::Mat IMU2GimbalT = (cv::Mat_<double>(4, 4) <<        //[TODO] IMU到枪管的相对位置未测量
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 1);


    cv::Mat Cam2IMUR = (cv::Mat_<double>(3, 3) <<
                        0, -1, 0,
                        0, 0, -1,
                        1, 0, 0);

    cv::Mat Cam2IMUt = (cv::Mat_<double>(1, 3) <<        //[TODO] 相机到IMU的相对位置未测量
                        0, 0, 0);



    cv::Mat Cam2IMUT = (cv::Mat_<double>(4, 4) <<           //[TODO] 相机到IMU的相对位置未测量
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 1);
    double IMUToBarrelX;
    double IMUToBarrelY;
    double IMUToBarrelZ;

    double initV;     // m/s
    double initK;
    double gravity;   // m/(s^2)
    double scaleDist;

    double kalmanDt;  // ms

    double windmillPitchAdd;  // degree
    double windmillYawAdd;    // degree

} poseSolverConfig;





bool InitConfigs(const std::string& pathToConfig, const std::string& pathToCalibFile)
{
    cv::FileStorage fsReader(pathToConfig, cv::FileStorage::READ);
    if (!fsReader.isOpened()) {
        return false;
    }

    robotName = (std::string)fsReader["robotName"];

    cv::FileNode poseSolverNode = fsReader["poseSolver"];
    poseSolverConfig.gravity = (double)poseSolverNode["gravity"];
    poseSolverConfig.initK = (double)poseSolverNode["initK"];
    poseSolverConfig.initV = (double)poseSolverNode["initV"];
    poseSolverConfig.scaleDist = (double)poseSolverNode["scaleDist"];
    poseSolverConfig.kalmanDt = (double)poseSolverNode["kalmanDt"];
    poseSolverConfig.windmillPitchAdd = (double)poseSolverNode["windmillPitchAdd"];
    poseSolverConfig.windmillYawAdd = (double)poseSolverNode["windmillYawAdd"];

    cv::FileNode aimbotNode = fsReader["aimbot"];
    aimbotConfig.areaNormalizedBase = (double)aimbotNode["areaNormalizedBase"];
    aimbotConfig.armorBigArmorRatio = (double)aimbotNode["armorBigArmorRatio"];
    aimbotConfig.armorMaxAngleDiff = (double)aimbotNode["armorMaxAngleDiff"];
    aimbotConfig.armorMaxAspectRatio = (double)aimbotNode["armorMaxAspectRatio"];
    aimbotConfig.armorMaxHeightDiffRatio = (double)aimbotNode["armorMaxHeightDiffRatio"];

    aimbotConfig.armorMaxYDiffRatio = (double)aimbotNode["armorMaxYDiffRatio"];
    aimbotConfig.armorMinAspectRatio = (double)aimbotNode["armorMinAspectRatio"];
    aimbotConfig.armorMinXDiffRatio = (double)aimbotNode["armorMinXDiffRatio"];
    aimbotConfig.armorSmallArmorRatio = (double)aimbotNode["armorSmallArmorRatio"];
    aimbotConfig.lightBarContourMinSolidity = (double)aimbotNode["lightBarContourMinSolidity"];

    aimbotConfig.lightBarEllipseMinAspectRatio = (double)aimbotNode["lightBarEllipseMinAspectRatio"];
    aimbotConfig.lightBarEllipseMaxAspectRatio = (double)aimbotNode["lightBarEllipseMaxAspectRatio"];
    aimbotConfig.lightBarMinArea = (double)aimbotNode["lightBarMinArea"];
    aimbotConfig.lightBarMaxArea = (double)aimbotNode["lightBarMaxArea"];
    aimbotConfig.sightOffsetNormalizedBase = (double)aimbotNode["sightOffsetNormalizedBase"];
    aimbotConfig.thresBrightness = (int)aimbotNode["thresBrightness"];
    aimbotConfig.thresColor = (int)aimbotNode["thresColor"];

    aimbotConfig.continousFireCount = (int)aimbotNode["continousFireCount"];

    aimbotConfig.fireBiasYawSmall = (double)aimbotNode["fireBiasYawSmall"];
    aimbotConfig.fireBiasPitchSmall = (double)aimbotNode["fireBiasPitchSmall"];
    aimbotConfig.fireBiasYawBig = (double)aimbotNode["fireBiasYawBig"];
    aimbotConfig.fireBiasPitchBig = (double)aimbotNode["fireBiasPitchBig"];

    cv::FileNode windmillNode = fsReader["windmill"];
    windmillConfig.thresBrightness = (int)windmillNode["thresBrightness"];
    windmillConfig.thresColor = (int)windmillNode["thresColor"];

    windmillConfig.armorMinArea = (double)windmillNode["armorMinArea"];
    windmillConfig.armorMaxArea = (double)windmillNode["armorMaxArea"];
    windmillConfig.armorMinAspectRatio = (double)windmillNode["armorMinAspectRatio"];
    windmillConfig.armorMaxAspectRatio = (double)windmillNode["armorMaxAspectRatio"];

    windmillConfig.centerMinArea = (double)windmillNode["centerMinArea"];
    windmillConfig.centerMaxArea = (double)windmillNode["centerMaxArea"];
    windmillConfig.minRadius = (double)windmillNode["minRadius"];
    windmillConfig.maxRadius = (double)windmillNode["maxRadius"];

    windmillConfig.timeDelaySmallBuff = (double)windmillNode["timeDelaySmallBuff"];
    windmillConfig.timeDelayBigBuff = (double)windmillNode["timeDelayBigBuff"];

    windmillConfig.fireBiasYaw = (double)windmillNode["fireBiasYaw"];
    windmillConfig.fireBiasPitch = (double)windmillNode["fireBiasPitch"];

    fsReader.release();

    fsReader.open(pathToCalibFile, cv::FileStorage::READ);
    if (!fsReader.isOpened()) {
        return false;
    }
    fsReader["CamMat"] >> imageLoaderConfig.cameraMatrix;
    fsReader["DistCoeffs"] >> imageLoaderConfig.distCoeffs;

    fsReader.release();

    /*std::cout << cameraMatrix << std::endl;
    std::cout << distCoeffs << std::endl;*/

    return true;
}

//[TODO]config路径统一管理