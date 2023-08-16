//Constants for Policy

#pragma once

#include "bits/stdc++.h"

#pragma pack(push, 1)
typedef struct RecvDateHP      //36
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

}DateHP;
#pragma pack(pop)