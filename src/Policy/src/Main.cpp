// Copyright (c) 2023 JiangMingyi
// Licensed under the MIT License.

#include "ros/ros.h"
#include "bits/stdc++.h"
#include "SerialDriver/SerialHP.h"
#include "SerialDriver/SerialBase.h"
#include "SerialDriver/SerialStamped.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Core"
#include "Policy/SpeedDate.h"
#include "Constants.hpp"
#include "Navigation.hpp"



int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"Policy");
    ros::NodeHandle node("Policy_Sys");
    ros::Publisher Speed_msg = node.advertise<Policy::SpeedDate>("/Policy/SpeedDate", 100);

    SerialDriver::SerialHP HPDate;
    Policy::SpeedDate speedDate;

    speedDate.ForwardSpeed = 0;
    speedDate.LeftSpeed = 0;
    speedDate.Chassis_angle = 0;

    Eigen::Vector3d Position, Pose;
    Eigen::Vector3d Rount[10][10][10];
    int Rount_Point_Num[10][10];
    bool whi_add = 0;

    std::vector<Eigen::Vector3d> TagPoint;
    int NO_Point = 0;
    int NO_Rount = 0;
    int NO_TagPoint1;
    int NO_TagPoint2;
    int My_TagPoint;
    int My_HP, outpose_HP, My_Color;

    int game_progress;
    int stage_remain_time;
    int bullet_remaining_num_17mm;


    Rount[1][2][0] << 0,0,0;      //任务点
    Rount[1][2][1] << 0,0,0;
    Rount[1][2][2] << 0,0,0;
    Rount[1][2][3] << 0,0,0;
    Rount_Point_Num[1][2] = 4;

    Rount[2][1][0] << 0,0,0;      //任务点
    Rount[2][1][1] << 0,0,0;
    Rount[2][1][2] << 0,0,0;
    Rount[2][1][3] << 0,0,0;
    Rount_Point_Num[2][1] = 4;

    for (int i = 0; i < 3; i++)
    {
        TagPoint.push_back(Rount[1][2][i]);
    }

    ros::Subscriber Date_msg = node.subscribe<SerialDriver::SerialStamped>("/SerialDriver/Date", 100, [&](const SerialDriver::SerialStamped::ConstPtr& Date)
    {
        My_Color = Date->Color;
    });

    ros::Subscriber HP_msg = node.subscribe<SerialDriver::SerialHP>("/SerialDriver/sendHP", 100, [&](const SerialDriver::SerialHP::ConstPtr& HPdate)
    {
        // memcpy(&HPDate, &sendHP, sizeof(HPDate));
        // RecvdateHP.
        if(My_Color = 1)
        {
            My_Color = HPdate->red_7_robot_HP;
            outpose_HP = HPdate->red_outpost_HP;
        }
        else
        {
            My_Color = HPdate->blue_7_robot_HP;
            outpose_HP = HPdate->blue_outpost_HP;
        }

    });

    ros::Subscriber Base_msg = node.subscribe<SerialDriver::SerialBase>("/SerialDriver/Base", 100, [&](const SerialDriver::SerialBase::ConstPtr& Basedate)
    {
        // memcpy(&HPDate, &sendHP, sizeof(HPDate));
        game_progress = Basedate->game_progress;
        stage_remain_time = Basedate->stage_remain_time;
        bullet_remaining_num_17mm = Basedate->bullet_remaining_num_17mm;
    });

    ros::Subscriber Lio_msg = node.subscribe<nav_msgs::Odometry>("/Odometry/R", 100, [&](const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        Position << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
        Pose << atan2(2 * (odom_msg->pose.pose.orientation.x * odom_msg->pose.pose.orientation.w + odom_msg->pose.pose.orientation.y * odom_msg->pose.pose.orientation.z), 1 - 2 * (odom_msg->pose.pose.orientation.x * odom_msg->pose.pose.orientation.x + odom_msg->pose.pose.orientation.y * odom_msg->pose.pose.orientation.y)), asin(2 * (odom_msg->pose.pose.orientation.w * odom_msg->pose.pose.orientation.y - odom_msg->pose.pose.orientation.z * odom_msg->pose.pose.orientation.x)), atan2(2 * (odom_msg->pose.pose.orientation.w * odom_msg->pose.pose.orientation.z + odom_msg->pose.pose.orientation.x * odom_msg->pose.pose.orientation.y), 1 - 2 * (odom_msg->pose.pose.orientation.y * odom_msg->pose.pose.orientation.y + odom_msg->pose.pose.orientation.z * odom_msg->pose.pose.orientation.z));

        if(!Position_Target(Rount[NO_TagPoint1][NO_TagPoint2][NO_Point], Position) && NO_Point <= Rount_Point_Num[NO_TagPoint1][NO_TagPoint2])
        {
            Target_Direction(Rount[NO_TagPoint1][NO_TagPoint2][NO_Point], Position, Pose[2] * 180 / Pi, speedDate.Chassis_angle);
            speedDate.ForwardSpeed = 3000 * ((180 - fabs(speedDate.Chassis_angle)) / 180) * ((180 - fabs(speedDate.Chassis_angle)) / 180) * ((180 - fabs(speedDate.Chassis_angle)) / 180);;
            speedDate.LeftSpeed = 0;
            speedDate.TopMode = 1;
            whi_add = false;
        }
        else if(Position_Target(Rount[NO_TagPoint1][NO_TagPoint2][NO_Point], Position) && NO_Point <= Rount_Point_Num[NO_TagPoint1][NO_TagPoint2])
        {
            speedDate.ForwardSpeed = 0;
            speedDate.LeftSpeed = 0;
            speedDate.Chassis_angle = 0;
            speedDate.TopMode = 1;
            if(whi_add == false)
            {
                whi_add = true;
                NO_Point++;
            }
        }
        else
        {
            speedDate.TopMode = 2;
            Road_Selection(game_progress, stage_remain_time, bullet_remaining_num_17mm, NO_TagPoint1, NO_TagPoint2, My_TagPoint, NO_Point, My_HP, outpose_HP);
        }
        Speed_msg.publish(speedDate);
        std::cout << speedDate.Chassis_angle << std::endl;
    });



    ros::spin();


    // ros::Subscriber Base_msg = node.subscribe<SerialDriver::

}