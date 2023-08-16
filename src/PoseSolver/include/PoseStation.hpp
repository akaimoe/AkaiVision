#pragma once

#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include <Eigen/LU>
#include "bits/stdc++.h"
#include "ros/ros.h"

class PoseStation
{
private:
    std::deque<cv::Mat> StationArmor;
    cv::Mat ArmorCentor;
    cv::Mat PreArmorCentor;
    Eigen::Vector3d PoseStaionCentor;
    int Speed;        // 1:正常旋转 2:减速旋转
    float Radius;     // 拟合出前哨站站装甲板旋转半径
    float Dist2Center;    // 装甲板距拟合前哨站中心的实际距离
    bool donePredict = 0;
    float OldAngle;
    float NowAngle;
    float PreAngle;
    int ArmorNO = 0;
    int Rotation_direction;     // 前哨站旋转方向 1：顺时针，2：逆时针

public:
    PoseStation(/* args */);
    ~PoseStation();
    void Add_Armor(cv::Mat TraMat);
    void Motion_Fit();
    void Motion_Prediction();
    bool Process(cv::Mat TraMat, cv::Mat &PoseStationTraMat, bool smodel_switch, int speed, int Rotation_direction);
};

PoseStation::PoseStation(/* args */)
{
}

PoseStation::~PoseStation()
{
}

void PoseStation::Add_Armor(cv::Mat TraMat)
{
    if(StationArmor.size() >= 100)
        StationArmor.pop_front();
    // ArmorCentor << TraMat.at<float>(0), TraMat.at<float>(1), TraMat.at<float>(2);
    ArmorCentor = TraMat;
    StationArmor.push_back(TraMat);
}

void PoseStation::Motion_Fit()
{
    // M 拟合点坐标
    // A 平面法向量
    // B 拟合坐标差值
    // C 圆心坐标
    std::vector<float> circle;
    int num = 100;
    int dim = 3;

    Eigen::MatrixXd M(num, dim);
    for (int i = 0; i < num; i++)
    {
        M(i, 0) = StationArmor[i].at<double>(0);
        M(i, 1) = StationArmor[i].at<double>(1);
        M(i, 2) = StationArmor[i].at<double>(2);

    }

    Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(num, 1);

    Eigen::MatrixXd A = (M.transpose()*M).inverse()*M.transpose()*L1;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num - 1, 3);

    for (int i = 0; i < num - 1; i++)
    {
        B.row(i) = M.row(i + 1) - M.row(i);
    }

    Eigen::MatrixXd L2 = Eigen::MatrixXd::Zero(num - 1, 1);
    for (int i = 0; i < num - 1; i++)
    {
        L2(i) = (M(i + 1, 0)*M(i + 1, 0) + M(i + 1, 1)*M(i + 1, 1) + M(i + 1, 2)*M(i + 1, 2)
                 - (M(i, 0)*M(i, 0) + M(i, 1)*M(i, 1) + M(i, 2)*M(i, 2))) / 2.0;
    }

    Eigen::MatrixXd D;
    //！！！矩阵合并前需要将合并后的矩阵 resize
    D.resize(4, 3);
    D << B.transpose()*B,
    A.transpose();

    Eigen::MatrixXd L3;
    Eigen::MatrixXd One31 = Eigen::MatrixXd::Ones(3, 1);
    L3.resize(4, 1);
    L3 << B.transpose()*L2,
    One31;

    // Eigen::MatrixXd C = (D.transpose()*D).inverse()*D.transpose() * L3;

    // double radius = 0;
    // for (int i = 0; i < num; i++)
    // {
    //     Eigen::MatrixXd tmp = M.row(i) - C.transpose();
    //     radius = radius + sqrt(tmp(0)*tmp(0) + tmp(1)*tmp(1) + tmp(2)*tmp(2));
    // }
    // radius = radius / num;

    // circle.push_back(C(0));
    // circle.push_back(C(1));
    // circle.push_back(C(2));
    // circle.push_back(A(0));
    // circle.push_back(A(1));
    // circle.push_back(A(2));
    // circle.push_back(radius);

    // PoseStaionCentor = C;
    // Radius = radius;

}

void PoseStation::Motion_Prediction()
{
    NowAngle = std::atan2((float)(PoseStaionCentor[0] - ArmorCentor.at<double>(0)), (float)(PoseStaionCentor[1] - ArmorCentor.at<double>(1)));
    if(Speed == 1)
    {
        if(Rotation_direction == 1)
            PreAngle = NowAngle + 10;
        else if(Rotation_direction == 2)
            PreAngle = NowAngle - 10;
    }
    else if(Speed == 2)
    {
        if(Rotation_direction == 1)
            PreAngle = NowAngle + 5;
        else if(Rotation_direction == 2)
            PreAngle = NowAngle - 5;
    }
    if(PreAngle >= 180)
        PreAngle -= 360;
    if(PreAngle <= -180)
        PreAngle += 360;
    PreArmorCentor.at<double>(0) = PoseStaionCentor[0] + Radius * std::sin(PreAngle);
    PreArmorCentor.at<double>(0) = PoseStaionCentor[1] + Radius * std::cos(PreAngle);
    PreArmorCentor.at<double>(2) = ArmorCentor.at<double>(2);
}

bool PoseStation::Process(cv::Mat TraMat, cv::Mat &PoseStationTraMat, bool model_switch, int speed, int rotation_direction)
{
    Speed = speed;
    Rotation_direction = rotation_direction;
    if(!model_switch)
    {
        donePredict = 0;
        StationArmor.clear();
        return donePredict;
    }
    if(StationArmor.size() < 100)
    {
        Add_Armor(TraMat);
        ROS_WARN("拟合未完成");
        return false;
    }
    else if(donePredict == 0)
    {
        Motion_Fit();
        donePredict = 1;
    }

    Motion_Prediction();
    PoseStationTraMat = PreArmorCentor;
    ROS_INFO("拟合完成");
    return donePredict;

}
