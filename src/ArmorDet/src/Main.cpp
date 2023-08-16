// Copyright (c) 2023 JiangMingyi
// Licensed under the MIT License.

#include <bits/stdc++.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <SerialDriver/SerialStamped.h>
#include <PoseSolver/Armor2DStamped.h>
#include "ros/ros.h"

#include "../include/Constants.hpp"
#include "../include/ArmorFound.hpp"
// #include "../include/Utils.hpp"
#include <tf/tf.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
// #include "../include/Windmill.hpp"

// #include "../include/PoseSolverTest.hpp"

void SolveRt2T(const cv::Mat &R, const cv::Mat &t, cv::Mat &T)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            T.at<double>(i, j) = R.at<double>(i, j);
        }
        T.at<double>(i, 3) = t.at<double>(0, i);
    }
}

void SolverPrint(const cv::Mat &T, int m)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            std::cout << T.at<double>(i, j) << '\t';
        }
        std::cout << std::endl;
    }
    std::cout << "------------------------------" << std::endl;
}

class ImagePLCGroup
{
public:
    ImagePLCGroup()
    {
        ros::NodeHandle node("ImagePLCGroup");
        armorPub = node.advertise<PoseSolver::Armor2DStamped>("ArmorDet/armor2d", 100);

        message_filters::Subscriber<sensor_msgs::Image> imageSub(node, "/hikrobot_camera/rgb", 100);
        message_filters::Subscriber<SerialDriver::SerialStamped> serialSub(node, "/SerialDriver/Date", 100);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, SerialDriver::SerialStamped> ImagePLCSyncPolicy;
        message_filters::Synchronizer<ImagePLCSyncPolicy> sync(ImagePLCSyncPolicy(100), imageSub, serialSub);
        sync.registerCallback(&SyncCallback);

        ros::spin();
    }

    static void SyncCallback(const sensor_msgs::ImageConstPtr &msgImage, const SerialDriver::SerialStamped &msgSerial)
    {
        try
        {
            cv::Mat img = cv_bridge::toCvShare(msgImage, "bgr8")->image;
            ros::Time stampImage = cv_bridge::toCvShare(msgImage, "bgr8")->header.stamp;
            FrameInfo frameinfo{img, stampImage};
            pAimbot->ColorSet(msgSerial.Color);
            // pWindmill->ColorSet(msgSerial.Color);

            int visionMode = 1; // read from msgSerial
            if (visionMode == 1)
            {
                // std::cout<<stampImage<<'\t'<< msgSerial.header.stamp<<std::endl;
                cv::Mat m_World2Camr, m_World2Camt, m_World2CamR;
                cv::Mat T1 = cv::Mat::eye(4, 4, CV_64FC1), T2 = cv::Mat::eye(4, 4, CV_64FC1), T3 = cv::Mat::eye(4, 4, CV_64FC1), T4 = cv::Mat::eye(4, 4, CV_64FC1), R3 = cv::Mat::eye(3, 3, CV_64FC1);
                Armor armor;
                if (pAimbot->Process(frameinfo, armor))
                {
                    msgArmor2D.header.stamp = msgSerial.header.stamp;

                    msgArmor2D.x1 = armor.Vertexes()[0].x;
                    msgArmor2D.y1 = armor.Vertexes()[0].y;

                    msgArmor2D.x2 = armor.Vertexes()[1].x;
                    msgArmor2D.y2 = armor.Vertexes()[1].y;

                    msgArmor2D.x3 = armor.Vertexes()[2].x;
                    msgArmor2D.y3 = armor.Vertexes()[2].y;

                    msgArmor2D.x4 = armor.Vertexes()[3].x;
                    msgArmor2D.y4 = armor.Vertexes()[3].y;

                    msgArmor2D.id = armor.Id();
                    msgArmor2D.type = (int)armor.Type();

                    msgArmor2D.oriention = msgSerial.q;

                    msgArmor2D.FoundArm = 0;

                    armorPub.publish(msgArmor2D);


                    /*****************************************发包部分**************************************/


                }
                else
                {
                    msgArmor2D.FoundArm++;
                    armorPub.publish(msgArmor2D);
                }
            }
            // else if(visionMode == 2 || visionMode == 3)
            // {
            //     cv::Point2d windmillCenter;
            //     if(pWindmill->Process(frameinfo, windmillCenter, visionMode))
            //     {
            //         msgArmor2D.header.stamp = msgSerial.header.stamp;
            //         msgArmor2D.x1 = windmillCenter.x;
            //         msgArmor2D.y1 = windmillCenter.y;

            //         if(msgSerial.Color == 1)
            //             msgArmor2D.id = 9;
            //         else
            //             msgArmor2D.id = 10;

            //         msgArmor2D.type = (int)ArmorType::Windmill;

            //         msgArmor2D.oriention = msgSerial.q;

            //         msgArmor2D.FoundArm = 0;

            //         armorPub.publish(msgArmor2D);

            //     }
            //     else
            //     {
            //         msgArmor2D.type = (int)ArmorType::Windmill;
            //         msgArmor2D.FoundArm++;
            //         armorPub.publish(msgArmor2D);
            //     }


            // }

            /*else if(visionT4T4T4T4T4T4T4T4T4T4T4T4T4
              pWindmill->Process(frameinfo);*/
#ifdef DEBUG
            cv::imshow("debug", img);
            cv::waitKey(1);
            // cout<<"tes"<<endl;
#endif //DEBUG
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msgImage->encoding.c_str());
        }
    }

private:
    // ModeProcessor *pointer;
    static Aimbot *pAimbot;
    // static Windmill *pWindmill;

    static ros::Publisher armorPub;
    static PoseSolver::Armor2DStamped msgArmor2D;

};
Aimbot *ImagePLCGroup::pAimbot = new Aimbot(1);
// Windmill *ImagePLCGroup::pWindmill = new Windmill();
PoseSolver::Armor2DStamped ImagePLCGroup::msgArmor2D;
ros::Publisher ImagePLCGroup::armorPub;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ArmorDet");
    if (!InitConfigs("/home/altair/ros/AkaiVisionROS/configs/RobotConfig.json", "/home/altair/ros/AkaiVisionROS/configs/MonoCalibResults.json"))
    {
        ROS_ERROR("Read config.json & MonoCalibResults.json failed ArmorDetNode");
        std::exit(-1);
    }

    ImagePLCGroup groupSubscriber;
}
