#include "Eigen/Core"

#define Pi 3.1415736


bool Position_Target(Eigen::Vector3d target, Eigen::Vector3d My_Pose)   //判断哨兵是否到到任务点附近
{
    double Mistake;
    return abs(target[0] - My_Pose[0]) <= Mistake && abs(target[1] - My_Pose[1]) <= Mistake && abs(target[2] - My_Pose[2]) <= Mistake;
}

void Target_Direction(Eigen::Vector3d target, Eigen::Vector3d My_Pose, double My_Dir, double &direction)
{
    direction = My_Dir - atan2(target[0]-My_Pose[0], target[1]-My_Pose[1]) * 180/Pi;     //计算哨兵需要转向相对角
    std::cout << "****" << My_Dir << "****" << atan2(target[0]-My_Pose[0], target[1]-My_Pose[1]) * 180/Pi << std::endl;
}

bool new_Point(int Point1, int Point2)
{
    return !(Point1 == Point2);
}

// 路径决策部分，通过比赛时间，剩余发弹量，自身血量和前哨站血量来决策移动策略
void Road_Selection(int game_progress, int stage_remain_time, int bullet_remaining_num_17mm, int &NO_TagPoint1, int &NO_TagPoint2, int &My_TagPoint, int &NO_Point, int My_HP, int outpose_HP)
{
    if(game_progress == 4)
    {
        if(bullet_remaining_num_17mm <= 100 || outpose_HP <= 100 || My_HP <= 100)  //特殊回城条件
        {
            if(new_Point(NO_TagPoint2, 1))   //判断目标点是否刷新
            {
                NO_Point = 0;
            }
            NO_TagPoint1 = My_TagPoint;
            NO_TagPoint2 = 1;
            My_TagPoint = 1;
            return;
        }
        if(stage_remain_time >=360)    //正常巡航规划
        {
            if(new_Point(NO_TagPoint2, 2))
            {
                NO_Point = 0;
            }
            NO_TagPoint1 = 1;
            NO_TagPoint2 = 2;
            My_TagPoint = 2;
            return;
        }
        if(stage_remain_time < 360 && stage_remain_time >= 180)
        {
            if(new_Point(NO_TagPoint2, 1))
            {
                NO_Point = 0;
            }
            NO_TagPoint1 = 2;
            NO_TagPoint2 = 1;
            My_TagPoint = 1;
            return;
        }
        if(stage_remain_time < 180 && stage_remain_time >= 0)
        {
            if(new_Point(NO_TagPoint2, 3))
            {
                NO_Point = 0;
            }
            NO_TagPoint1 = 1;
            NO_TagPoint2 = 3;
            My_TagPoint = 3;
            return;
        }
    }
}