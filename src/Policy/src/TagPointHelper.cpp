// Copyright (c) 2023 JiangMingyi
// Licensed under the MIT License.

#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"
#include<opencv2/highgui.hpp>
#include "Eigen/Core"

cv::Mat *img = 0;

double map_x, map_y, img_x, img_y;     //出发点402， 410（像素坐标）0，0（实际坐标）
//284-537  5023  1：19.8537

void onMouse(int event, int x, int y, int flags, void* param)
{
    cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
    switch (event)
    {
    case 1:     //鼠标左键按下响应：返回坐标和灰度
        std::cout << "at(" << x << "," << y << ")value is:"
                  << static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;
        map_x = (x-402)*19.8537/1000;
        map_y = (y-410)*19.8537/1000;
        std::cout << "map_point:" << map_x << " " << map_y << std::endl;
        break;
    case 2:    //鼠标右键按下响应：输入坐标并返回该坐标的灰度
        // std::cout << "input(x,y)" << std::endl;
        // std::cout << "x =" << std::endl;
        // std::cin >> x;
        // std::cout << "y =" << std::endl;
        // std::cin >> y;
        // std::cout << "at(" << x << "," << y << ")value is:"
        //           << static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;
        break;
    }
}

int main()
{

    cv::Mat src = cv::imread("/home/altair/img/map.png");
    img = &src;
    cv::Mat src2 = src.clone();
    cv::namedWindow("original image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("original image", onMouse, reinterpret_cast<void*> (img));//注册鼠标操作(回调)函数
    imshow("original image", src);

    cv::waitKey();
    return 0;

}
