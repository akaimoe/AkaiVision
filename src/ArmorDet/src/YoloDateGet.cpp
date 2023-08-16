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
#include "../include/ArmorFound.hpp"
#include "../include/Constants.hpp"
// #include "../include/Utils.hpp"
#include <tf/tf.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
// #include "../include/Windmill.hpp"

// #include "../include/PoseSolverTest.hpp"

// #define SAVE_IMG

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "YoloDateGet");

    if (!InitConfigs("/home/altair/ros/AkaiVisionROS/configs/RobotConfig.json", "/home/altair/ros/AkaiVisionROS/configs/MonoCalibResults.json"))
    {
        ROS_ERROR("Read config.json & MonoCalibResults.json failed ArmorDetNode");
        std::exit(-1);
    }
    cv::VideoCapture vid_capture("/home/altair/item/armor_tag_data/outpost.avi");   //视频输入路径
    if (!vid_capture.isOpened())
    {
        ROS_ERROR("Error opening video stream or file");
        return 0;
    }
    else
    {
        int fps = vid_capture.get(5);
        int frame_count = vid_capture.get(7);
    }
    cv::Mat img;
    Aimbot pAimbot;
    Armor TagArmor;
    FrameInfo frame;
    std::string SavePathVal = "/home/altair/item/armor_tag_data/SaveImg/val/";     //数据保存路径
    std::string SavePathTrain = "/home/altair/item/armor_tag_data/SaveImg/train/";
    std::string SavePathTest = "/home/altair/item/armor_tag_data/SaveImg/test/";
    std::string SaveName;
    pAimbot.ColorSet(2);     //1 for blue; 2 for red
    int num = 1;
    int i = 1;
    int j = 1;
    while(vid_capture.isOpened())
    {
        bool isSuccess = vid_capture.read(img);
        if(isSuccess == true)
        {

            frame.frame = img;
            if(pAimbot.Process(frame, TagArmor))
            {
                try
                {
                    cv::Mat CutImg = img(TagArmor.BBox());
                    cv::rectangle(img, TagArmor.BBox(), cv::Scalar(255), 1);
                    // cv::drawContours(img, )
                    // std::cout << TagArmor.BoundingBox().x << ',' << TagArmor.BoundingBox().y << "  " << TagArmor.BoundingBox().height << ',' << TagArmor.BoundingBox().width << std::endl;
                    cv::resize(CutImg, CutImg, cv::Size(640, 640), 0, 0, cv::INTER_NEAREST);
                    cv::cvtColor(CutImg, CutImg, CV_BGR2GRAY);
                    cv::imshow("CutFrame", CutImg);
#ifdef SAVE_IMG
                    if(num <= 3)
                    {
                        SaveName = SavePathTrain + "8/train_" + std::to_string(j) + ".png";
                        j++;
                    }
                    else
                    {
                        SaveName = SavePathVal + "8/val_" + std::to_string(i) + ".png";
                        i++;
                        num = 0;
                    }
                    cv::imwrite(SaveName, CutImg);
#endif
                    num++;
#ifdef DEBUG
                    std::cout << j << "   " << i << std::endl;
#endif
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }


            }
            cv::imshow("Frame", img);

        }
        int key = cv::waitKey(20);
        if (key == 'q')
        {
            ROS_INFO("q key is pressed by the user. Stopping the video");
            break;
        }
    }

}
