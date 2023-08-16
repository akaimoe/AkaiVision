// Copyright (c) 2023 JiangMingyi
// Licensed under the MIT License.

#include<bits/stdc++.h>
#include "ros/ros.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
#include <tf/tf.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/PointStamped.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <SerialDriver/SerialStamped.h>
#include "SerialDriver/SerialSend.h"
#include <PoseSolver/Armor2DStamped.h>
#include <../include/KalmanFilter.hpp>
#include <../include/PoseStation.hpp>
#include"../include/PoseSolver.hpp"
using namespace std;


void drawround(double& x, double& y, double& z, double len, double rant)
{
    x = len * cos(rant);
    y = len * sin(rant);
    z = 0;
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"PoseSolve");
    ros::NodeHandle node("Pose_KF");

    if (!InitConfigs("/home/altair/ros/AkaiVisionROS/configs/RobotConfig.json", "/home/altair/ros/AkaiVisionROS/configs/MonoCalibResults.json"))
    {
        ROS_ERROR("Read config.json & MonoCalibResults.json failed PoseSolverNode");
        std::exit(-1);
    }

    //PoseSolver::Armor2DStamped armorVer;

    double rant = 0;        //测试数据

    PoseSolve poseSolver;
    SerialDriver::SerialSend serialSend;
    bool FlagFound, FlagFire;
    std::vector<cv::Point2f> Vertexes(4, cv::Point2f());
    double yaw, pitch;
    cv::Mat TraMat = (cv::Mat_<double>(1, 3) << (0, 0, 0)), IMU_Recv = cv::Mat::eye(3, 3, CV_64FC1), predicted;
    KalmanFilter kalmanFilter(9,6);
    ros::Publisher PosePub = node.advertise<geometry_msgs::PointStamped> ("PosePub", 1);
    ros::Publisher PosePub_kal = node.advertise<geometry_msgs::PointStamped> ("PosePub_kal", 1);
    ros::Publisher sendMsg = node.advertise<SerialDriver::SerialSend>("/PoseSolver/sendMsg", 100);
    ros::Subscriber armorSub = node.subscribe<PoseSolver::Armor2DStamped>("ArmorDet/armor2d", 100, [&](const PoseSolver::Armor2DStamped::ConstPtr& armorMsg)
    {
        Vertexes[0].x = armorMsg->x1;
        Vertexes[0].y = armorMsg->y1;

        Vertexes[1].x = armorMsg->x2;
        Vertexes[1].y = armorMsg->y2;

        Vertexes[2].x = armorMsg->x3;
        Vertexes[2].y = armorMsg->y3;

        Vertexes[3].x = armorMsg->x4;
        Vertexes[3].y = armorMsg->y4;

        FlagFound = armorMsg->FoundArm <= aimbotConfig.continousFireCount;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(armorMsg->oriention, quat);
        tf::Matrix3x3 R(quat);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                IMU_Recv.at<double>(i, j) = R[i][j];
            }
        }

        if(armorMsg->type != 3)
        {
            poseSolver.Solve(Vertexes, armorMsg->id, TraMat, IMU_Recv);
            if(fabs(TraMat.at<float>(2)) > 1500)
                return;
            /**************************************数据测试*************************************/
            // for (int i = 0; i < 1; i++)
            // {
            //     TraMat.at<double>(i) += 0.5;
            // }
            // drawround(TraMat.at<double>(0), TraMat.at<double>(1), TraMat.at<double>(2), 30, rant);
            // rant += 0.03;
            /**************************************数据测试*************************************/

            predicted = kalmanFilter.PredictAndCorrect(TraMat, armorMsg->header.stamp);
            double px = predicted.at<float>(0);
            double py = predicted.at<float>(1);
            double pz = predicted.at<float>(2);

            geometry_msgs::PointStamped PoseMsg, PoseMsg_kal;
            PoseMsg.header.frame_id = "turtle1";
            PoseMsg.header.stamp = armorMsg->header.stamp;
            PoseMsg.point.x = TraMat.at<double>(0);
            PoseMsg.point.y = TraMat.at<double>(1);
            PoseMsg.point.z = TraMat.at<double>(2);

            PoseMsg_kal.header.frame_id = "turtle1";
            PoseMsg_kal.header.stamp = armorMsg->header.stamp;
            PoseMsg_kal.point.x = px;
            PoseMsg_kal.point.y = py;
            PoseMsg_kal.point.z = pz;

            std::cout<<px<<"\t"<<py<<"\t"<<pz<<std::endl<<TraMat.at<double>(0)<<"\t"<<TraMat.at<double>(1)<<"\t"<<TraMat.at<double>(2)<<std::endl<<std::endl;

            // poseSolver.SolveCompensation(predicted, yaw, pitch);    //打开卡尔曼预测
            poseSolver.SolveCompensation(TraMat, yaw, pitch);          //关闭卡尔曼预测

            if(armorMsg->type == 1)
                FlagFire = fabs(yaw) < aimbotConfig.fireBiasYawBig && fabs(pitch) < aimbotConfig.fireBiasPitchBig;
            else if(armorMsg->type == 2)
                FlagFire = fabs(yaw) < aimbotConfig.fireBiasYawSmall && fabs(pitch) < aimbotConfig.fireBiasPitchSmall;

            geometry_msgs::PointStamped point_base;
            geometry_msgs::PointStamped point_base_kal;
            // point_base = tfss.transform(PoseMsg,"world");
            // point_base_kal = tfs.transform(PoseMsg_kal,"world");

            PosePub.publish(PoseMsg);
            //PosePub.publish(PoseMsg_kal);
            PosePub_kal.publish(PoseMsg_kal);
        }
        else
        {
            if(FlagFound)
            {
                cv::Point2d windmill;
                windmill.x = (double)Vertexes[0].x;
                windmill.y = (double)Vertexes[0].y;
                double dist = 7000.0;
                poseSolver.SolveCompensationWindmill(windmill, 27.5, yaw, pitch, dist);
                FlagFire = fabs(yaw) < windmillConfig.fireBiasYaw && fabs(pitch) < windmillConfig.fireBiasPitch;
            }
        }
        if(FlagFound)
        {
            serialSend.FrameHead = 0xA5;
            serialSend.Pitch = pitch;
            serialSend.Yaw = yaw;
            serialSend.FlagFound = 1;
            serialSend.FlagFire = FlagFire;
            serialSend.FrameTail = 0xAA;
        }
        else
        {
            serialSend.FrameHead = 0xA5;
            serialSend.Pitch = 0;
            serialSend.Yaw = 0;
            serialSend.FlagFound = 0;
            serialSend.FlagFire = 0;
            serialSend.FrameTail = 0xAA;
        }

        sendMsg.publish(serialSend);
    });

    ros::spin();

}