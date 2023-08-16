#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/persistence.hpp>

#include <iostream>
#include <iomanip>
#include <functional>
#include <chrono>
#include <cstdlib>
#include <sstream>
#include <cctype>
#include <iomanip>

#ifdef _WIN32
#pragma warning(disable : 4996)
#endif // _WIN32

std::string GetTimestampStr()
{
    std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //std::tm tm = *std::gmtime(&tt); //GMT (UTC)
    std::tm tm = *std::localtime(&tt); //Locale time-zone, usually UTC by default.
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
    return ss.str();
}

cv::Mat GetTransFormMatrix(const cv::Mat& r, const cv::Mat& tvec, bool isRVec)
{
    cv::Mat R;
    if (isRVec)
        cv::Rodrigues(r, R);
    else
        R = r;

    cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            T.at<double>(i, j) = R.at<double>(i, j);
        }
    }

    for (size_t i = 0; i < 3; ++i)
    {
        T.at<double>(i, 3) = tvec.at<double>(i);
    }

    return T;
}

void GetRotTrans(const cv::Mat& T, cv::Mat& r, cv::Mat& tvec, bool toRVec)
{
    cv::Mat R(3, 3, CV_64FC1, cv::Scalar(0));
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            R.at<double>(i, j) = T.at<double>(i, j);
        }
    }
    if (toRVec)
        cv::Rodrigues(R, r);
    else
        r = R.clone();

    tvec = cv::Mat::zeros(1, 3, CV_64FC1);
    for (size_t i = 0; i < 3; ++i)
    {
        tvec.at<double>(i) = T.at<double>(i, 3);
    }

}

class MonoCalibrator
{
public:
    struct MonocularParams
    {
        cv::Mat cameraMatrix; //相机内参矩阵K 3*3
        cv::Mat distCoeffs;   //相机的5个畸变系数矩阵：k1,k2,p1,p2,k3 1*5

        //相机外参（相对于标定板）
        std::vector<cv::Mat> rvecs; //每幅图像的旋转向量
        std::vector<cv::Mat> tvecs; //每幅图像的平移向量

        double rmse;
    };

public:
    MonoCalibrator() = default;
    MonoCalibrator(int imgWidth, int imgHeight) :
        m_ImageWidth(imgWidth), m_ImageHeight(imgHeight), m_ImgSize(imgWidth, imgHeight) {}

    void SetChessboardParams(int numCornerRow, int numCornerCol, int cellWidth, int cellHeight) //11 8 25 25
    {
        m_BoardSize = cv::Size(numCornerRow, numCornerCol);
        m_BoardCellSize = cv::Size(cellWidth, cellHeight);
    }

    bool CalibProc(const std::vector<cv::Mat>& images, MonocularParams& camParams)
    {
        m_Images = images;
        //找棋盘角点
        if (!ExtractCorners())
        {
            std::cerr << "Cannot Find Chessboard Corners!" << std::endl; //找不到角点
            return false;
        }

        CalibMonocular(camParams);

        return true;
    }

    void EvaluateMonocularCalibResults(const MonocularParams& camParams)
    {
        size_t imgCnt = camParams.rvecs.size();
        double total_err = 0.0; //所有图像的平均误差的总和
        std::vector<cv::Point2f> image_points2; /* 保存重新计算得到的投影点 */
        std::cout << "每幅图像的标定误差：" << std::endl;

        for (size_t i = 0; i < imgCnt; ++i)
        {
            std::vector<cv::Point3f> tempPointSet = m_ObjectPoints[i];
            /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
            projectPoints(tempPointSet, camParams.rvecs[i], camParams.tvecs[i], camParams.cameraMatrix, camParams.distCoeffs, image_points2);
            /* 计算新的投影点和旧的投影点之间的误差*/
            cv::Mat tempImagePointMat = cv::Mat(1, m_ImagesCorners[i].size(), CV_32FC2);
            cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);

            for (size_t j = 0; j < m_ImagesCorners[i].size(); ++j)
            {
                image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
                tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(m_ImagesCorners[i][j].x, m_ImagesCorners[i][j].y);
            }

            double err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2); //每幅图像的平均误差
            total_err += err /= (m_BoardSize.width * m_BoardSize.height);
            std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << std::endl;
        }

        std::cout << "总体rms误差：" << total_err / imgCnt << "像素" << std::endl << std::endl;
    }

    void WriteResultToFile(const std::string& camId, const MonocularParams& camParams, bool saveImages)
    {
        std::string timestamp = GetTimestampStr();

#ifdef _WIN32
        std::string parentDir = std::string("..\\mono_calib_data\\") + timestamp + std::string("\\");
        std::string imgSaveDir = parentDir + "images\\";
#else
        std::string parentDir = std::string("../mono_calib_data/") + timestamp + std::string("/");
        std::string imgSaveDir = parentDir + "images/";
#endif // _WIN32 

        std::system(cv::format("mkdir %s", parentDir.c_str()).c_str());

        if (saveImages)
        {
            std::stringstream ss;
            ss << "mkdir " << imgSaveDir;
            std::system(ss.str().c_str());
            ss.str("");

            for (size_t i = 0; i < m_Images.size(); ++i)
            {
                ss << imgSaveDir << std::setw(2) << std::setfill('0') << i << ".png";
                cv::imwrite(ss.str(), m_Images[i]);
                ss.str("");
            }
        }

        cv::FileStorage fsWriter(parentDir + std::string("MonoCalibResults.json"), cv::FileStorage::WRITE);
        fsWriter << "CamId" << camId;
        fsWriter << "Calib_Date" << timestamp;
        fsWriter << "Image_Size" << m_ImgSize;
        fsWriter << "Root_Mean_Squared_Error" << camParams.rmse;
        fsWriter << "CamMat" << camParams.cameraMatrix;
        fsWriter << "DistCoeffs" << camParams.distCoeffs;
        fsWriter.release();
    }

    const std::vector<std::vector<cv::Point3f>>& ObjectPoints()
    {
        return m_ObjectPoints;
    }

    const std::vector<std::vector<cv::Point2f>>& ImagesCorners()
    {
        return m_ImagesCorners;
    }

private:
    int m_ImageWidth;
    int m_ImageHeight;
    cv::Size m_ImgSize;  //单目图像尺寸
    cv::Size m_BoardSize;  // 标定板上每行、列的角点数
    cv::Size m_BoardCellSize; //实际测量得到的标定板上每个棋盘格的大小 mm

    std::vector<cv::Mat> m_Images;
    std::vector<std::vector<cv::Point2f>> m_ImagesCorners;
    std::vector<std::vector<cv::Point3f>> m_ObjectPoints;

private:
    bool ExtractCorners()
    {
        std::vector<cv::Point2f> corners;  //存储每幅图像上检测到的角点
        m_ImagesCorners.clear();
        for (auto image : m_Images)
        {
            //提取角点
            if (cv::findChessboardCorners(image, m_BoardSize, corners))
            {
                //对粗提取到的角点进行亚像素精确化
                cv::Mat gray;
                cvtColor(image, gray, cv::COLOR_RGB2GRAY);

                find4QuadCornerSubpix(gray, corners, cv::Size(11, 11));
                m_ImagesCorners.emplace_back(corners); //保存亚像素角点
                /* 在图像上显示角点位置 */
                drawChessboardCorners(image, m_BoardSize, corners, true);
                imshow("Camera Calibration", image);
                cv::waitKey(50);
            }
            else
            {
                std::cerr << "Cannot Find Chessboard Corners!" << std::endl; //找不到角点
                cv::imshow("Bad Image", image);
                cv::waitKey(0);
                return false;
            }
        }

        return true;
    }

    void CalibMonocular(MonocularParams& camParams)
    {
        //初始化标定板上角点的三维坐标
        m_ObjectPoints.clear();
        for (size_t k = 0; k < m_Images.size(); ++k)
        {
            std::vector<cv::Point3f> tempPointsSet;

            for (size_t i = 0; i < m_BoardSize.height; ++i)
            {
                for (size_t j = 0; j < m_BoardSize.width; ++j)
                {
                    cv::Point3f realPoint;
                    /* 假设标定板放在世界坐标系中z=0的平面上 */
                    realPoint.x = i * m_BoardCellSize.width;
                    realPoint.y = j * m_BoardCellSize.height;
                    realPoint.z = 0;
                    tempPointsSet.push_back(realPoint);
                }
            }

            m_ObjectPoints.push_back(tempPointsSet);
        }

        camParams.rmse =
            cv::calibrateCamera(m_ObjectPoints, m_ImagesCorners, m_ImgSize, camParams.cameraMatrix, camParams.distCoeffs,
                                camParams.rvecs, camParams.tvecs);
        std::cout << "单目标定误差：" << camParams.rmse << std::endl;
    }
};

class StereoCalibrator
{
public:
    enum CamPosition
    {
        Left = 0,
        Right
    };
    struct StereoParams
    {
        cv::Mat R, t; //右目到左目的旋转矩阵与平移向量
        cv::Mat T;    //右目到左目的欧氏变换阵
        cv::Mat E;    //本质矩阵
        cv::Mat F;    //基础矩阵

        double rmse;
    };

public:
    StereoCalibrator(int imgWidth, int imgHeight) :
        m_ImageWidth(imgWidth), m_ImageHeight(imgHeight), m_ImgSize(imgWidth, imgHeight)
    {
        m_MonoCalibrators[Left] = MonoCalibrator(imgWidth, imgHeight);
        m_MonoCalibrators[Right] = MonoCalibrator(imgWidth, imgHeight);
    }

    void SetChessboardParams(int numCornerRow, int numCornerCol, int cellWidth, int cellHeight) //11 8 25 25
    {
        m_MonoCalibrators[Left].SetChessboardParams(numCornerRow, numCornerCol, cellWidth, cellHeight);
        m_MonoCalibrators[Right].SetChessboardParams(numCornerRow, numCornerCol, cellWidth, cellHeight);
    }

    bool CalibProc(
        const std::vector<cv::Mat>& imagesLeft,
        const std::vector<cv::Mat>& imagesRight,
        StereoParams& stereoParams)
    {
        m_CamsImages[Left] = imagesLeft;
        m_CamsImages[Right] = imagesRight;

        if (!m_MonoCalibrators[Left].CalibProc(imagesLeft, m_CamsParams[Left])
                || !m_MonoCalibrators[Right].CalibProc(imagesRight, m_CamsParams[Right]))
            return false;

        CalibStereo(stereoParams,
                    m_MonoCalibrators[Left].ObjectPoints(),
        std::array< std::vector<std::vector<cv::Point2f>>, 2> {
            m_MonoCalibrators[Left].ImagesCorners(), m_MonoCalibrators[Right].ImagesCorners()
        });

        return true;
    }

    void WriteResultToFile(const StereoParams& stereoParams)
    {
        std::string timestamp = GetTimestampStr();

#ifdef _WIN32
        std::string parentDir = std::string("CalibData\\") + timestamp + std::string("\\");
        std::string imgSaveDirLeft = parentDir + "imgL\\";
        std::string imgSaveDirRight = parentDir + "imgR\\";
#else
        std::string parentDir = std::string("CalibData/") + timestamp + std::string("/");
        std::string imgSaveDirLeft = parentDir + "imgL/";
        std::string imgSaveDirRight = parentDir + "imgR/";
#endif // _WIN32 

        std::stringstream ss;
        ss << "mkdir " << imgSaveDirLeft;
        std::system(ss.str().c_str());
        ss.str("");
        ss << "mkdir " << imgSaveDirRight;
        std::system(ss.str().c_str());
        ss.str("");

        for (size_t i = 0; i < m_CamsImages.size(); ++i)
        {
            for (size_t j = 0; j < m_CamsImages[i].size(); ++j)
            {
                if (i == 0)
                {
                    ss << imgSaveDirLeft << "L_" << std::setw(2) << std::setfill('0') << j << ".png";
                    cv::imwrite(ss.str(), m_CamsImages[i][j]);
                    ss.str("");
                }

                else if (i == 1)
                {
                    ss << imgSaveDirRight << "R_" << std::setw(2) << std::setfill('0') << j << ".png";
                    cv::imwrite(ss.str(), m_CamsImages[i][j]);
                    ss.str("");
                }
            }
        }

        cv::FileStorage fsWriter(parentDir + std::string("StereoCalibResults.json"), cv::FileStorage::WRITE);
        fsWriter << "CamId" << "";
        fsWriter << "Calib_Date" << timestamp;
        fsWriter << "Image_Size" << m_ImgSize;
        fsWriter << "Root_Mean_Squared_Error" << stereoParams.rmse;
        fsWriter << "CamMat_Left" << m_CamsParams[Left].cameraMatrix;
        fsWriter << "DistCoeffs_Left" << m_CamsParams[Left].distCoeffs;
        fsWriter << "CamMat_Right" << m_CamsParams[Right].cameraMatrix;
        fsWriter << "DistCoeffs_Right" << m_CamsParams[Right].distCoeffs;
        fsWriter << "Stereo_E" << stereoParams.E;
        fsWriter << "Stereo_F" << stereoParams.F;
        fsWriter << "Stereo_T" << stereoParams.T;
        fsWriter << "Stereo_R" << stereoParams.R;
        fsWriter << "Stereo_t" << stereoParams.t;
        fsWriter.release();
    }

private:
    int m_ImageWidth;
    int m_ImageHeight;
    cv::Size m_ImgSize;  //单目图像尺寸
    std::array<MonoCalibrator, 2> m_MonoCalibrators;
    std::array<MonoCalibrator::MonocularParams, 2> m_CamsParams;
    std::array<std::vector<cv::Mat>, 2> m_CamsImages;

private:
    void CalibStereo(
        StereoParams& stereoParams,
        const std::vector<std::vector<cv::Point3f>>& objectPoints,
        const std::array<std::vector<std::vector<cv::Point2f>>, 2>& camsBoardCorners)
    {
        //此函数参数的cam1为右目，cam2为左目
        stereoParams.rmse =
            cv::stereoCalibrate(objectPoints,
                                camsBoardCorners[Right], camsBoardCorners[Left],
                                m_CamsParams[Right].cameraMatrix, m_CamsParams[Right].distCoeffs,
                                m_CamsParams[Left].cameraMatrix, m_CamsParams[Left].distCoeffs,
                                m_ImgSize,
                                stereoParams.R, stereoParams.t,
                                stereoParams.E, stereoParams.F);

        /*std::cout << stereoParams.R << std::endl;
        cv::Mat sumT_cr2cl(4, 4, CV_64FC1, cv::Scalar(0));
        for (size_t i = 0; i < m_CamsParams[Left].rvecs.size(); ++i)
        {
            cv::Mat T_w2cl = GetTransFormMatrix(m_CamsParams[Left].rvecs[i], m_CamsParams[Left].tvecs[i], true);
            cv::Mat T_w2cr = GetTransFormMatrix(m_CamsParams[Right].rvecs[i], m_CamsParams[Right].tvecs[i], true);
            sumT_cr2cl += T_w2cl * T_w2cr.inv();
        }
        cv::Mat T_cr2cl = sumT_cr2cl / m_CamsParams[Left].rvecs.size();
        GetRotTrans(T_cr2cl, stereoParams.R, stereoParams.t, false);
        std::cout << stereoParams.R << std::endl;*/
        stereoParams.T = GetTransFormMatrix(stereoParams.R, stereoParams.t, false);
        std::cout << "双目标定rms重投影误差：" << stereoParams.rmse << std::endl;
    }
};
