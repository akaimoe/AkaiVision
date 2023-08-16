// Copyright (c) 2023 JiangMingyi
// Licensed under the MIT License.

#include "ros/ros.h"
#include "../include/Constants.hpp"
#include "serial/serial.h"
#include "SerialDriver/SerialStamped.h" //普通文本类型的消息
#include "SerialDriver/SerialSend.h"
#include "SerialDriver/SerialHP.h"
#include "SerialDriver/SerialBase.h"
#include "Policy/SpeedDate.h"
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <tf/tf.h>
// #include < tf2_geometry_msgs.h>


int main(int argc, char  *argv[])
{

    setlocale(LC_ALL,"");

    ros::init(argc,argv,"SerialDriver");

    ros::NodeHandle node("SerialDriver");
    ros::Publisher StampedPub = node.advertise<SerialDriver::SerialStamped>("/SerialDriver/Date",10);
    ros::Publisher BasePub = node.advertise<SerialDriver::SerialBase>("/SerialDriver/Base", 10);
    ros::Publisher HPPub = node.advertise<SerialDriver::SerialHP>("/SerialDriver/HP", 10);
    uint8_t sendbuffer[26], recvbuffer[39], headbuffer;    //接收数据为43位
    SendDate sendDate;
    RecvDate recvDate;
    RecvDateBase recvDateBase;
    RecvDateHP recvDateHP;
    serial::Serial serial;
    try
    {
        serial.setPort("/dev/ttyACM0");
        serial.setBaudrate(12000000);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial.setTimeout(timeout);
        serial.open();
        ROS_INFO("串口连上了喵~");
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("打不开串口了喵~ :(");
    }


    SerialDriver::SerialStamped StampedMsg;
    SerialDriver::SerialBase BaseMsg;
    SerialDriver::SerialHP HPMsg;

#ifdef ROS_IMU_TEST
    ros::Subscriber imuSub = node.subscribe<sensor_msgs::Imu>("/handsfree/imu", 100, [&](const sensor_msgs::Imu::ConstPtr& imuMsg) {
        // 把sensor_msgs::Imu里面的orientation四元数转成欧拉角roll，pitch，yaw
        ROS_INFO("----------------");
        msg.q = imuMsg->orientation;
        msg.header.stamp = imuMsg->header.stamp;
        msg.Color = 2;
        msg.TargetType = 1;      //此处根据实际情况赋值，目前此时默认为1，即装甲板模式，若测试风车性能，此处参数改为2
        pub.publish(msg);
    });
#endif //ROS_IMU_TEST

    ros::Subscriber posesolverSub = node.subscribe<SerialDriver::SerialSend>("/PoseSolver/sendMsg", 100, [&](const SerialDriver::SerialSend::ConstPtr& sendMsg)
    {
        sendDate.Pitch = sendMsg->Pitch;
        sendDate.Yaw = sendMsg->Yaw;
        std::cout<<sendDate.Pitch<<"--------"<<sendDate.Yaw<<std::endl;
        // sendDate.FlagFound = sendMsg->FlagFound;
        // sendDate.FlagFire = sendMsg->FlagFire;
        sendDate.FlagFire = sendMsg->FlagFire;
        // sendDate.FireMode = 0;
        // sendDate.ForwardSpeed = 0;
        // sendDate.LeftSpeed = 0;
        // sendDate.Chassis_angle = 0;
        // for (int i = 0; i < sizeof(sendbuffer); i++)
        // {
        //     std::cout << std::hex << (sendbuffer[i] & 0xff) << " ";
        // }
        // std::cout<<std::endl;

    });

    ros::Subscriber PolicySub = node.subscribe<Policy::SpeedDate>("/Policy/SpeedDate", 100, [&](const Policy::SpeedDate::ConstPtr& PolicyMsg)
    {
        sendDate.ForwardSpeed = PolicyMsg->ForwardSpeed;
        sendDate.LeftSpeed = PolicyMsg->LeftSpeed;
        sendDate.Chassis_angle = PolicyMsg->Chassis_angle + recvDateBase.yaw;
        sendDate.TopMode = PolicyMsg->TopMode;
        memcpy(sendbuffer, &sendDate, sizeof(sendDate));
#ifdef SERIAL_READ
        serial.write(sendbuffer, sizeof(sendbuffer));
#endif //SERIAL_READ
    });


    // ros::MultiThreadedSpinner spinner(3);

    ros::Rate r(500);

    while (ros::ok())
    {

        r.sleep();
#ifdef SERIAL_READ
        serial.read(&headbuffer, sizeof(headbuffer));
        if(headbuffer == 0xA5)
        {
            serial.read(&headbuffer, sizeof(headbuffer));
            if(headbuffer == 0xB1)
            {
                serial.read(&headbuffer, sizeof(headbuffer));
                if(headbuffer == 0xB2)
                {
                    serial.read(&headbuffer, sizeof(headbuffer));
                    if(headbuffer == 0xB3)
                    {
                        serial.read(recvbuffer, sizeof(recvbuffer));
                        if(recvbuffer[38] != 0xAA)
                        {
                            ROS_ERROR("帧尾错误喵~");
                            continue;
                        }
                        memcpy(&recvDateBase, recvbuffer, sizeof(RecvDateBase));
                        // std::cout<<"------------------"<<recvDateBase.IMURoll<<' '<<recvDateBase.IMUPitch<<' '<<recvDateBase.IMUYaw<<std::endl;
                        // std::cout<<"#############"<<recvDateBase.MyColor<<std::endl;
                        StampedMsg.header.stamp = ros::Time::now();
                        StampedMsg.q = tf::createQuaternionMsgFromRollPitchYaw(recvDateBase.IMURoll, recvDateBase.IMUPitch, recvDateBase.IMUYaw);
                        StampedMsg.Color = recvDateBase.MyColor;
                        BaseMsg.bullet_remaining_num_17mm = recvDateBase.bullet_remaining_num_17mm;
                        BaseMsg.game_progress = recvDateBase.game_progress;
                        BaseMsg.Seat = recvDateBase.Seat;
                        BaseMsg.stage_remain_time = recvDateBase.stage_remain_time;
                        BaseMsg.x = recvDateBase.x;
                        BaseMsg.y = recvDateBase.y;
                        BaseMsg.z = recvDateBase.z;

                        BasePub.publish(BaseMsg);
                        StampedPub.publish(StampedMsg);

                    }
                }
            }

        }
        else if(headbuffer == 0xA6)
        {
            serial.read(&headbuffer, sizeof(headbuffer));
            if(headbuffer == 0xB1)
            {
                serial.read(&headbuffer, sizeof(headbuffer));
                if(headbuffer == 0xB2)
                {
                    serial.read(&headbuffer, sizeof(headbuffer));
                    if(headbuffer == 0xB3)
                    {
                        serial.read(recvbuffer, sizeof(recvbuffer));
                        if(recvbuffer[38] != 0xAA)
                        {
                            ROS_ERROR("帧尾错误喵~");
                            continue;
                        }
                        memcpy(&recvDateHP, recvbuffer, sizeof(RecvDateHP));
                        // ROS_INFO("收到HP的串口数据了喵~");
                        memcpy(&HPMsg, &recvDateHP, sizeof(HPMsg));
                        HPPub.publish(HPMsg);

                    }
                }
            }
        }
        else
        {
            StampedPub.publish(StampedMsg);
        }
        // serial.read(recvbuffer, sizeof(recvbuffer));
        // for (int i = 0; i < sizeof(recvbuffer); i++)
        // {
        //     if(recvbuffer[i] == 0xA5 && recvbuffer[i+39] == 0xAA)
        //         // if(1)
        //     {
        //         memcpy(&recvDate, recvbuffer+i, sizeof(RecvDate));
        //         // ROS_INFO("收到Base的串口数据了喵~");
        //         memcpy(&recvDateBase, recvDate.Buffer, sizeof(RecvDateBase));
        //         // msg.TargetType = recvDateBase.VisionMode;
        //         // std::cout<<"------------------"<<recvDateBase.IMURoll<<' '<<recvDateBase.IMUPitch<<' '<<recvDateBase.IMUYaw<<std::endl;
        //         // std::cout<<"#############"<<recvDateBase.MyColor<<std::endl;
        //         msg.header.stamp = ros::Time::now();
        //         msg.q = tf::createQuaternionMsgFromRollPitchYaw(recvDateBase.IMURoll, recvDateBase.IMUPitch, recvDateBase.IMUYaw);
        //         msg.Color = recvDateBase.MyColor;
        //         pub.publish(msg);
        //         i+=39;
        //     }
        //     else if(recvbuffer[i] == 0xA6 && recvbuffer[i+39] == 0xAA)
        //     {
        //         // ROS_INFO("收到HP的串口数据了喵~");
        //         memcpy(&recvDate, recvbuffer+i, sizeof(RecvDate));
        //         memcpy(&recvDateHP, recvDate.Buffer, sizeof(RecvDateHP));
        //         i+=39;
        //     }
        // }


        // for (int i = 0; i < sizeof(recvbuffer); i++)
        // {
        //     std::cout << std::hex << (recvbuffer[i] & 0xff) << " ";
        // }
        // std::cout<<std::endl;


        // ROS_INFO("收到串口数据了喵~");
        // std::cout<<sizeof(RecvDate)<<' '<<sizeof(recvbuffer)<<std::endl;


#endif //SERIAL_READ
//暂无应用
        ros::spinOnce();
    }
// ros::spin();


    return 0;
}