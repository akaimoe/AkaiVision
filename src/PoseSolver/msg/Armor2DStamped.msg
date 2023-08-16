Header header

float64 x1
float64 y1
float64 x2
float64 y2
float64 x3
float64 y3
float64 x4
float64 y4

int32 id # include color and number 
int32 type #  0 for none, 1 for big, 2 for small armor,3 for windmill

geometry_msgs/Quaternion oriention

int32 FoundArm

# ID对照表：
# 0： NONE
#
# 1： 红方英雄
# 2： 红方工程
# 3： 红方一号步兵
# 4： 红方二号步兵
# 5： 红方三号步兵
# 6： 红方哨兵
# 7： 红方前哨站
# 8： 红方基地
# 9： 红色能量机关（蓝方击打）
# 
# 11：蓝方英雄
# 12：蓝方工程
# 13：蓝方一号步兵
# 14：蓝方二号步兵
# 15：蓝方三号步兵
# 16：蓝方哨兵
# 17：蓝方前哨站
# 18：蓝方基地
# 19：蓝色能量机关（红方击打）