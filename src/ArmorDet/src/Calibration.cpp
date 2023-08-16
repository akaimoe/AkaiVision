#include <opencv2/core/utility.hpp>
#include "../include/Calibration.hpp"

constexpr int kImageWidth = 1440;
constexpr int kImageHeight = 1080;

int main()
{
    std::string camId;
    std::cout << "Input Camera Name: ";
    std::cin >> camId;

    std::vector<std::string> imageNames;
    cv::glob("../calib_images/", imageNames);   //设置标定图片路径
    std::vector<cv::Mat> images;
    for (size_t i = 0; i < imageNames.size(); ++i)
    {
        std::cout << imageNames[i] << std::endl;
        images.push_back(cv::imread(imageNames[i]));
    }

    MonoCalibrator::MonocularParams camParams;
    MonoCalibrator monoCalibrator(kImageWidth, kImageHeight);
    monoCalibrator.SetChessboardParams(11, 8, 25, 25);           //标定板点位数量、尺寸
    if (monoCalibrator.CalibProc(images, camParams))
    {
        monoCalibrator.EvaluateMonocularCalibResults(camParams);
    }
    else
    {
        std::cerr << "Calibration ERROR" << std::endl;
    }
    monoCalibrator.WriteResultToFile(camId, camParams, false);

    /*cv::FileStorage fsReader("CalibResults", cv::FileStorage::READ);
    cv::Mat a; fsReader["camM"] >> a;
    cv::Mat b; fsReader["distC"] >> b;
    std::cout << a << b;*/
    ///*
    //cv::Mat src = imread("WIN_20190920_13_22_20_Pro.jpg");
    //cv::Mat dst;
    //undistort(src, dst, cameraMatrix, distCoeffs);
    //imshow("undistort", dst);
    //waitKey(0);
    //*/
    return 0;
}