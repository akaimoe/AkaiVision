//Constants for SerialDriver

#pragma once

#define ROS_IMU_TEST
#define SERIAL_READ


#ifdef SERIAL_READ
#undef ROS_IMU_TEST
#endif

#include "bits/stdc++.h"

#pragma pack(push, 1)      //26
struct SendDate
{
    uint8_t FrameHead = 0xA5;
    /************当TopMode为2时，以下数据生效**************/
    float Pitch;
    float Yaw;
    /***************************************************/
    // uint8_t FlagFound;
    uint8_t FlagFire;
    uint8_t FireMode;    //0:单发模式，1：连发模式
    uint8_t TopMode;  //1:导航模式，哨兵为底盘跟随模式，此模式下Pitch与Yaw数据不对云台进行控制；2：射击模式，底盘锁死/原地陀螺，云台通过Pitch与Yaw控制
    uint8_t Gyroscope;    //0:关闭陀螺，1：开启陀螺
    /************当TopMode为1时，以下数据生效**************/
    float ForwardSpeed;
    float LeftSpeed;
    float Chassis_angle;  //底盘旋转角度
    /***************************************************/
    uint8_t FrameTail = 0xAA;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RecvDateBase     // 35
{

    // float BulletSpeed;
    uint8_t MyColor;
    // uint8_t Race;   //兵种，1：英雄，2：步兵，3：哨兵
    uint8_t VisionMode;

    float IMURoll;
    float IMUPitch;
    float IMUYaw;

    float x;    //本机位置信息（UWB）
    float y;
    float z;
    float yaw;

    uint8_t game_progress;    //0:未开始，1:准备阶段，2：自检阶段，3：5s倒计时，4：对战中，5：结算
    uint16_t stage_remain_time;     //当前阶段剩余时间，单位 s
    uint16_t bullet_remaining_num_17mm;    //17mm弹丸剩余弹量
    uint32_t Seat; //1:R4环形高地，2：能量机关激活点，此数据用于验证哨兵是否已达到指定位置并被裁判系统识别

};
#pragma pack(pop)

#pragma pack(push, 1)
struct RecvDateHP      //36
//血量信息
{
    uint16_t red_1_robot_HP;   //英雄
    uint16_t red_2_robot_HP;   //工程
    uint16_t red_3_robot_HP;   //步兵
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;   //哨兵
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;

};
#pragma pack(pop)

#pragma pack(push, 1)
struct RecvDate          //50
{
    uint8_t FrameHead;
    // uint8_t DateType;     //1:RecvDateBase  2:RecvDateHP  3:RecvDateSys

    uint8_t Buffer[38];

    uint8_t FrameTail;
};
#pragma pack(pop)

