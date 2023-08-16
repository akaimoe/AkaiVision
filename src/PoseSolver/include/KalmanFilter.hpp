#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "Constants.hpp"
using namespace std;

// 限幅函数
template<typename T>
inline T Clamp(T value, const T& lowerBnd, const T& upperBnd)
{
    if (value > upperBnd)
        return upperBnd;
    else if (value < lowerBnd)
        return lowerBnd;
    else
        return value;
}


class KalmanFilter
{
private:
    int m_stateNum, m_measureNum;
    cv::KalmanFilter KF;
    cv::Mat measurement;

    double lastX{}, lastY{}, lastZ{};
    ros::Time lastTime;

public:
    KalmanFilter() {}
    KalmanFilter(int stateNum, int measureNum);
    ~KalmanFilter();
    void Init();
    cv::Mat PredictAndCorrect(const cv::Mat& curCoord, const ros::Time& timestamp);
};

KalmanFilter::KalmanFilter(int stateNum, int measureNum)
{
    m_stateNum = stateNum;
    m_measureNum = measureNum;
    KF = cv::KalmanFilter(m_stateNum, m_measureNum, 0);
    Init();
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init()
{
    lastX = lastY = lastZ = 0;
    lastTime = ros::Time(0);

    //Mat processNoise(stateNum, 1, CV_32F);
    measurement = cv::Mat::zeros(m_measureNum, 1, CV_32F);
    float dt = poseSolverConfig.kalmanDt;
    std::cout<<"dt="<<poseSolverConfig.kalmanDt<<std::endl;
    KF.transitionMatrix = (cv::Mat_<float>(m_stateNum, m_stateNum) <<    //A 状态转移矩阵
                           1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0,
                           0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0,
                           0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt,
                           0, 0, 0, 1, 0, 0, dt, 0, 0,
                           0, 0, 0, 0, 1, 0, 0, dt, 0,
                           0, 0, 0, 0, 0, 1, 0, 0, dt,
                           0, 0, 0, 0, 0, 0, 1, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 1, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 1);
    //KF.transitionMatrix = (cv::Mat_<float>(stateNum, stateNum) <<    //A 状态转移矩阵
    //    1, 0, dt, 0,
    //    0, 1, 0, dt,
    //    0, 0, 1, 0,
    //    0, 0, 0, 1);
    // 当我们要更信任观测值时，可以将观测噪声R调小；当我们要更信任估计值时，可以将R调大，也可以将Q调大
    //这里没有设置控制矩阵B，默认为零
    setIdentity(KF.measurementMatrix);								//H=[1,0,0,0;0,1,0,0] 测量矩阵
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-6));			//Q高斯白噪声，单位阵 过程噪声
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(100));		//R高斯白噪声，单位阵 测量噪声
    setIdentity(KF.errorCovPost, cv::Scalar::all(1));				//P后验误差估计协方差矩阵，初始化为单位阵
    randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));	//初始化前一时刻状态为随机值
}

cv::Mat KalmanFilter::PredictAndCorrect(const cv::Mat& curCoord, const ros::Time& timestamp)
{
    static constexpr double kMaxSpeed = 3; // 1m/s == 1mm/ms

    double vx{}, vy{}, vz{};  // mm/ms
    double curX = curCoord.at<double>(0), curY = curCoord.at<double>(1), curZ = curCoord.at<double>(2);

    if (lastTime == ros::Time(0))
    {
        // 新目标出现
        measurement = (cv::Mat_<float>(m_measureNum, 1) << curX, curY, curZ, 0, 0, 0);
    }
    else
    {
        double interval = (timestamp.toNSec() - lastTime.toNSec()) / 1000000.0; // ms
        vx = Clamp((curX - lastX) / interval, -kMaxSpeed, kMaxSpeed);
        vy = Clamp((curY - lastY) / interval, -kMaxSpeed, kMaxSpeed);
        vz = Clamp((curZ - lastZ) / interval, -kMaxSpeed, kMaxSpeed);
        ROS_INFO("t:{%lf}, vx:{%lf}, vy:{%lf}, vz:{%lf}", interval, vx, vy, vz);

        measurement = (cv::Mat_<float>(m_measureNum, 1) << curX, curY, curZ, vx, vy, vz);
    }
    lastTime = timestamp;
    lastX = curX;
    lastY = curY;
    lastZ = curZ;

    cv::Mat prediction = KF.predict();    //ERROR
    KF.correct(measurement);

    return prediction;
}

// class KalmanFilter
// {
// private:
//     int m_stateNum, m_measureNum;
//     cv::KalmanFilter KF;
//     cv::Mat measurement;

//     double lastX{}, lastY{}, lastZ{};
//     ros::Time lastTime;

// public:
//     KalmanFilter() {}
//     KalmanFilter(int stateNum, int measureNum);
//     ~KalmanFilter();
//     void Init();
//     cv::Mat PredictAndCorrect(const cv::Mat& curCoord, const ros::Time& timestamp);
// };

// KalmanFilter::KalmanFilter(int stateNum=6, int measureNum=3)
// {
//     m_stateNum = stateNum;
//     m_measureNum = measureNum;
//     KF = cv::KalmanFilter(m_stateNum, m_measureNum, 0);
//     Init();
// }

// KalmanFilter::~KalmanFilter()
// {
// }

// void KalmanFilter::Init()
// {
//     lastX = lastY = lastZ = 0;
//     lastTime = ros::Time(0);

//     //Mat processNoise(stateNum, 1, CV_32F);
//     measurement = cv::Mat::zeros(m_measureNum, 1, CV_32F);
//     float dt = poseSolverConfig.kalmanDt;
//     std::cout<<"dt="<<poseSolverConfig.kalmanDt<<std::endl;
//     KF.transitionMatrix = (cv::Mat_<float>(m_stateNum, m_stateNum) <<    //A 状态转移矩阵
//                            1, 0, 0, dt,0,0,
//                            0, 1, 0, 0,dt,0,
//                            0, 0, 1, 0,0,dt,
//                            0, 0, 0, 1,0,0);
//     // 当我们要更信任观测值时，可以将观测噪声R调小；当我们要更信任估计值时，可以将R调大，也可以将Q调大
//     //这里没有设置控制矩阵B，默认为零
//     setIdentity(KF.measurementMatrix);								//H=[1,0,0,0;0,1,0,0] 测量矩阵
//     setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));			//Q高斯白噪声，单位阵 过程噪声
//     setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));		//R高斯白噪声，单位阵 测量噪声
//     setIdentity(KF.errorCovPost, cv::Scalar::all(1));				//P后验误差估计协方差矩阵，初始化为单位阵
//     randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));	//初始化前一时刻状态为随机值
// }

// cv::Mat KalmanFilter::PredictAndCorrect(const cv::Mat& curCoord, const ros::Time& timestamp)
// {
//     double curX = curCoord.at<double>(0), curY = curCoord.at<double>(1), curZ = curCoord.at<double>(2);
//     measurement = (cv::Mat_<float>(m_measureNum, 1) << curX, curY, curZ);

//     KF.correct(measurement);
//     cv::Mat prediction = KF.predict();

//     return prediction;
// }