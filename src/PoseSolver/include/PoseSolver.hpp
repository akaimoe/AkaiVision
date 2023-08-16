#include<opencv2/opencv.hpp>
#include<bits/stdc++.h>
#include"Constants.hpp"


class PoseSolve
{
private:
    cv::Mat m_World2Camr, m_World2Camt, m_World2CamR, m_World2CamT = cv::Mat::eye(4, 4, CV_64FC1);      //t:平移向量 r：旋转向量 T：位姿矩阵 R：旋转矩阵
    // cv::Mat m_Cam2IMUr, m_Cam2IMUt;
    cv::Mat m_r, m_t, m_T = cv::Mat::eye(4, 4, CV_64FC1);
    cv::Mat m_RcIMU;
    cv::Mat m_PoseInGimbal;
    cv::Mat m_PoseInIMU;
    double m_Pitch, m_Yaw, m_Dist;
    ArmorType m_ArmorType;
    // cv::Mat m_IMURecv;
public:
    PoseSolve();
    ~PoseSolve();
    void PNPSolver(const std::vector<cv::Point2f>& vertex2d, ArmorType armorType);
    void IMUSolver();
    void Solve(const std::vector<cv::Point2f>& vertex2d, int ArmorTypeId, cv::Mat& TraMat, cv::Mat IMU_Recv);
    void SolveRt2T(const cv::Mat &R, const cv::Mat &t, cv::Mat &T);
    void SolveCompensation(cv::Mat predicted, double& yaw, double& pitch);
    bool SolveCompensationWindmill(const cv::Point2d& pt, double bulletSpeed, double& yaw, double& pitch, double& dist);
    double ModifyAimbotPitch(double pitch, double dist);
    double ModifyAimbotYaw(double Yaw, double dist);
    double ModifyWindmillPitch(double pitch, double dist);
    double ModifyWindmillYaw(double Yaw, double dist);
};


PoseSolve::PoseSolve()
{
}

PoseSolve::~PoseSolve()
{
}

void PoseSolve::SolveRt2T(const cv::Mat &R, const cv::Mat &t, cv::Mat &T)
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

void PoseSolve::PNPSolver(const std::vector<cv::Point2f>& vertex2d, ArmorType armorType)
{
    // cv::solvePnP参数：void solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags = CV_ITERATIVE)
    //      objectPoints - 世界坐标系下的控制点的坐标，vector的数据类型在这里可以使用
    //      imagePoints - 在图像坐标系下对应的控制点的坐标。vector在这里可以使用
    //      cameraMatrix - 相机的内参矩阵
    //      distCoeffs - 相机的畸变系数
    //      rvec - 输出的旋转向量。使坐标点从世界坐标系旋转到相机坐标系
    //      tvec - 输出的平移向量。使坐标点从世界坐标系平移到相机坐标系
    //      flags - 默认使用CV_ITERATIV迭代法
    m_ArmorType = armorType;
    switch (armorType)
    {
    case ArmorType::Small:
        cv::solvePnPRansac(POINT_3D_OF_ARMOR_SMALL, vertex2d, imageLoaderConfig.cameraMatrix, imageLoaderConfig.distCoeffs, m_World2Camr, m_World2Camt, false, 100, 8, 0.999, cv::noArray(), cv::SOLVEPNP_IPPE);
        break;
    case ArmorType::Big:
        cv::solvePnP(POINT_3D_OF_ARMOR_BIG, vertex2d, imageLoaderConfig.cameraMatrix, imageLoaderConfig.distCoeffs, m_World2Camr, m_World2Camt);
        break;
    case ArmorType::Windmill:
        cv::solvePnP(POINT_3D_OF_ARMOR_WINDMILL, vertex2d, imageLoaderConfig.cameraMatrix, imageLoaderConfig.distCoeffs, m_World2Camr, m_World2Camt);
    default:
        break;
    }

    cv::Rodrigues(m_World2Camr, m_World2CamR);
    //SolveRt2T(m_World2CamR, m_World2Camt, m_World2CamT);

#ifdef DEMARCATE
    m_T = m_World2CamT*poseSolverConfig.Cam2IMUT*m_IMURecv;
#else
    // m_T = m_World2CamT*poseSolverConfig.Cam2IMUT*m_IMURecv*poseSolverConfig.IMU2GimbalT;
    // std::cout<<m_World2Camt.size()<<std::endl<<poseSolverConfig.Cam2IMUR.size()<<std::endl<<poseSolverConfig.Cam2IMUt.size()<<std::endl<<m_RcIMU.size()<<std::endl;
    m_t = ((m_World2Camt.t()*poseSolverConfig.Cam2IMUR) + poseSolverConfig.Cam2IMUt)*(m_RcIMU.t());

#endif
    for (int i = 0; i < 3; i++)
    {
        std::cout<<m_t.at<double>(i)<<"\t";
    }
    std::cout<<std::endl;

    // m_PoseInGimbal = poseSolverConfig.ArmorCS * m_T;
    // static const cv::Mat kCam2IMU = (cv::Mat_<double>(3, 1) << poseSolverConfig.cameraToIMUX, poseSolverConfig.cameraToIMUY, poseSolverConfig.cameraToIMUZ);
    // m_Cam2IMUt = m_World2Camt + kCam2IMU;

    // static const cv::Mat kIMU2GblTran = (cv::Mat_<double>(3, 1) << poseSolverConfig.IMUToBarrelX, poseSolverConfig.IMUToBarrelY, poseSolverConfig.IMUToBarrelZ);
    // m_t = m_Cam2IMUt + kIMU2GblTran;

}


void PoseSolve::Solve(const std::vector<cv::Point2f>& vertex2d, int armorTypeId, cv::Mat& TraMat, cv::Mat IMU_Recv)
{
    ArmorType armorType;
    armorTypeId %= 10;
    if((armorTypeId >= 2 || armorTypeId <= 5)&& armorTypeId == 7)
        armorType = ArmorType::Small;
    else if(armorTypeId == 1 && armorTypeId == 6 && armorTypeId == 8)
        armorType = ArmorType::Big;
    else if(armorTypeId == 9)
        armorType = ArmorType::Windmill;
    m_RcIMU = IMU_Recv;

    PNPSolver(vertex2d, armorType);
    // m_PoseInGimbal = m_t;
    TraMat = m_t;
}

void PoseSolve::SolveCompensation(cv::Mat predicted, double& yaw, double& pitch)
{
    m_Yaw = atan2(predicted.at<double>(1), predicted.at<double>(0));
    m_Pitch = atan2(predicted.at<double>(2), predicted.at<double>(0));
    m_Dist = sqrt(predicted.at<double>(0)*predicted.at<double>(0) + predicted.at<double>(0)*predicted.at<double>(0) + predicted.at<double>(0)*predicted.at<double>(0));

    m_Yaw *= kRadToDegreeCoef;
    m_Pitch *= kRadToDegreeCoef;

    yaw = ModifyAimbotYaw(m_Yaw, m_Dist);
    pitch = ModifyAimbotPitch(m_Pitch, m_Dist);
    std::cout<<"posesolver测试节点"<<yaw<<' '<<pitch<<std::endl;
}


bool PoseSolve::SolveCompensationWindmill(const cv::Point2d& pt, double bulletSpeed, double& yaw, double& pitch, double& dist)
{
    static const double fx = imageLoaderConfig.cameraMatrix.at<double>(0, 0);
    static const double fy = imageLoaderConfig.cameraMatrix.at<double>(1, 1);
    static const double cx = imageLoaderConfig.cameraMatrix.at<double>(0, 2);
    static const double cy = imageLoaderConfig.cameraMatrix.at<double>(1, 2);
    cv::Point2f pnt;
    std::vector<cv::Point2f> in{ pt };
    std::vector<cv::Point2f> out;

    cv::undistortPoints(in, out, imageLoaderConfig.cameraMatrix, imageLoaderConfig.distCoeffs, cv::noArray(), imageLoaderConfig.cameraMatrix);
    pnt = out.front();

    double rxNew = (pnt.x - cx) / fx;
    double ryNew = (pnt.y - cy) / fy;

    yaw = atan(rxNew);
    pitch = -atan(ryNew);
    dist = 7000;

    // 相机---枪口偏移补偿
    // double camera_target_height = dist * sin(pitch);
    // double gun_target_height = camera_target_height + poseSolverConfig.cameraToBarrelY;
    // double gun_pitch_tan = gun_target_height / (dist * cos(pitch));
    // pitch = atan(gun_pitch_tan);

    // 简化的重力补偿
    //bulletSpeed=27;//写为定值27
    //bulletSpeed *= 1000.0;
    //double compensateGravity_pitch_tan = tan(pitch) + (0.5 * 9.8 * (dist / bulletSpeed) * (dist / bulletSpeed)) / cos(pitch);
    //pitch = atan(compensateGravity_pitch_tan);

    yaw *= kRadToDegreeCoef;
    pitch *= kRadToDegreeCoef;

    yaw = ModifyWindmillYaw(yaw, dist);
    pitch = ModifyWindmillPitch(pitch, dist);

    return IsLegalNum(yaw) && IsLegalNum(pitch) && IsLegalNum(dist);
}


/*************************************补偿函数拟合**********************************/

double PoseSolve::ModifyAimbotPitch(double pitch, double dist)
{
    double newPitch = pitch;
    newPitch+= 1.447e-08*dist*dist-0.0004343*dist+0.228;
    if(dist>9000||dist<1000||abs(newPitch)>50)
    {
        // newPitch=0;
    }
    return newPitch;

}

double PoseSolve::ModifyAimbotYaw(double yaw, double dist)
{
    double newYaw = yaw;
    //1522
    if(dist>1400&&dist<=1750)
        newYaw-=0.5;
    //1895 2339
    if(dist>1750&&dist<=2600)
        newYaw-=0.9;
    //2780
    if(dist>2600&&dist<=3000)
        newYaw-=1.2;
    //3130 3565 3800
    if(dist>3000&&dist<=4700)
        newYaw-=1.4;
    //4250 4560 4850
    if(dist>4700)
        newYaw-=1.2;
    //5200
    if(dist>9000||dist<1000||abs(newYaw)>50)
    {
        // newYaw=0;
    }

    return newYaw;

}


double PoseSolve::ModifyWindmillPitch(double pitch, double dist)
{
    double newPitch = pitch;
    newPitch+= 1.447e-08*dist*dist-0.0004343*dist+0.228;
    if(dist>9000||dist<1000||abs(newPitch)>50)
    {
        // newPitch=0;
    }
    return newPitch;

}

double PoseSolve::ModifyWindmillYaw(double yaw, double dist)
{
    double newYaw = yaw;
    //1522
    if(dist>1400&&dist<=1750)
        newYaw-=0.5;
    //1895 2339
    if(dist>1750&&dist<=2600)
        newYaw-=0.9;
    //2780
    if(dist>2600&&dist<=3000)
        newYaw-=1.2;
    //3130 3565 3800
    if(dist>3000&&dist<=4700)
        newYaw-=1.4;
    //4250 4560 4850
    if(dist>4700)
        newYaw-=1.2;
    //5200
    if(dist>9000||dist<1000||abs(newYaw)>50)
    {
        // newYaw=0;
    }

    return newYaw;

}



/*************************************补偿函数拟合**********************************/


