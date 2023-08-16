#include<opencv2/opencv.hpp>
#include<bits/stdc++.h>
#include"Constants.hpp"


// 此文件用于前期测试
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
    cv::Mat ArmorCS = (cv::Mat_<double>(3, 1) <<            //装甲板坐标系，默认以装甲板中心为目标点
                       0, 0, 0);

    cv::Mat IMU2GimbalT = (cv::Mat_<double>(4, 4) <<        //[TODO] IMU到枪管的相对位置未测量
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 1);

    cv::Mat Cam2IMUT = (cv::Mat_<double>(4, 4) <<           //[TODO] 相机到IMU的相对位置未测量
                        0, -1, 0, 0,
                        0, 0, -1, 0,
                        1, 0, 0,  0,
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