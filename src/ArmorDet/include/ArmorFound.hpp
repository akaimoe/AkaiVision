#pragma once

#include "bits/stdc++.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/dnn.hpp"
#include "Constants.hpp"
#include "time.h"

class Light
{
private:                                    //数据说明
    std::vector<cv::Point> m_LightContour;  //灯条轮廓
    cv::RotatedRect m_Ellipse;              //拟合矩形
    float m_Width;                          //短轴
    float m_Length;                         //长轴
    float m_Angle;                          //灯条角度
    float m_Area;                           //灯条拟合椭圆面积


public:
    Light(/* args */) {};
    Light(const std::vector<cv::Point>& lightContour, const cv::Rect& frameROIRect);
    ~Light() {};
    bool LightDetector();
    cv::RotatedRect Ellipse()
    {
        return m_Ellipse;
    }
    float Angle() const
    {
        return m_Angle;
    }
    float Length()
    {
        return m_Length;
    }
    cv::Point2f Center() const
    {
        return m_Ellipse.center;
    }
    float Area() const
    {
        return m_Area;
    }
};

Light::Light(const std::vector<cv::Point>& lightContour, const cv::Rect& frameROIRect)
{
    m_LightContour = lightContour;
    m_Ellipse = cv::fitEllipse(lightContour);
    m_Ellipse.center += cv::Point2f(frameROIRect.tl());
    m_Length = m_Ellipse.size.height;
    m_Width = m_Ellipse.size.width;
    m_Angle = m_Ellipse.angle;
    m_Area = m_Length / 2.0 * m_Width / 2.0 * CV_PI;

    while(m_Angle < -90)
    {
        m_Angle += 180;
    }
    while(m_Angle >= 90)
    {
        m_Angle -= 180;
    }
    if (m_Angle >= 45.0)
    {
        std::swap(m_Length, m_Width);
        m_Angle -= 90.0;
    }
    if (m_Angle < -45.0)
    {
        std::swap(m_Length, m_Width);
        m_Angle += 90.0;
    }
}

bool Light::LightDetector()
{
    float LightAspectRatio = m_Length / m_Width;
    if(LightAspectRatio < aimbotConfig.lightAspectRatioMax
            && LightAspectRatio > aimbotConfig.lightAspectRatioMin
            && m_Area < aimbotConfig.lightAreaMax
            && m_Area > aimbotConfig.lightAreaMin)
        return true;
    return false;
}

class Armor
{
private:
    std::string m_ModlePath;
    cv::dnn::Net m_Net;                     //yolo5 onnx
    Light m_RightLight;
    Light m_LeftLight;
    std::vector<cv::Point2f> m_Vertexes;    //装甲板四个顶点，顺时针排列
    float m_Area;
    cv::Point2f m_Center;                   //装甲板中心坐标
    float m_AngleDiff;
    float m_LenDiff;
    cv::Rect2d m_BBox;
    cv::Point2f m_LeftLightVertexes[4], m_RightLightVertexes[4];  //灯条拟合矩形四个顶点，顺时针排列，从左下开始
    ArmorType m_ArmorType;
    std::vector<cv::Mat> m_NumFrame;
    int m_id;
    double m_Target2CenterDist;

public:
    Armor() {};
    Armor(Light RightLight, Light LeftLight);
    ~Armor() {};
    bool ArmorDetector(const cv::Mat& frame);
    std::vector<cv::Point2f> Vertexes() const
    {
        return m_Vertexes;
    }
    int Id() const
    {
        return m_id;
    }
    cv::Rect2d BBox() const
    {
        return m_BBox;
    }
    cv::Point2f Center() const
    {
        return m_Center;
    }
    double Target2CenterDist() const
    {
        return m_Target2CenterDist;
    }
    ArmorType Type() const
    {
        return m_ArmorType;
    }

};

Armor::Armor(Light RightLight, Light LeftLight)
{
    cv::ocl::setUseOpenCL(false);

    m_ModlePath = aimbotConfig.modlePath;
    m_RightLight = RightLight;
    m_LeftLight = LeftLight;
#ifdef Yolo
    m_Net = cv::dnn::readNetFromONNX(m_ModlePath);
    std::cout << "in1" << std::endl;
#ifdef CUDA
    m_Net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    m_Net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
#else
    m_Net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    m_Net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
#endif

    if (m_Net.empty())
    {
        ROS_ERROR("未找到模型nia～，请检查模型路径是否正确");
        exit(-1);
    }
#endif
    m_AngleDiff = abs(m_RightLight.Angle() - m_LeftLight.Angle());
    if (m_AngleDiff >= 180)
        m_AngleDiff -= 180;
    else if (m_AngleDiff > 170)
        m_AngleDiff = 180 - m_AngleDiff;
    m_LenDiff = fabs(m_LeftLight.Length() - m_RightLight.Length()) / std::max(m_LeftLight.Length(), m_RightLight.Length());;

    m_RightLight.Ellipse().points(m_RightLightVertexes);
    m_LeftLight.Ellipse().points(m_LeftLightVertexes);


    m_Vertexes.push_back((m_LeftLightVertexes[0] + m_LeftLightVertexes[3]) / 2.0);      //左下
    m_Vertexes.push_back((m_LeftLightVertexes[1] + m_LeftLightVertexes[2]) / 2.0);      //左上
    m_Vertexes.push_back((m_RightLightVertexes[1] + m_RightLightVertexes[2]) / 2.0);      //右上
    m_Vertexes.push_back((m_RightLightVertexes[0] + m_RightLightVertexes[3]) / 2.0);      //右上

    m_BBox = cv::boundingRect(m_Vertexes);
    cv::Point2f LeftCenter = (m_LeftLightVertexes[0] + m_LeftLightVertexes[2]) / 2.0;
    cv::Point2f RightCenter = (m_RightLightVertexes[0] + m_RightLightVertexes[2]) / 2.0;
    m_Center = (LeftCenter + RightCenter) / 2.0;

    cv::Point2f frameCenter(imageConfig.frameHeight / 2, imageConfig.frameWidth / 2);
    m_Target2CenterDist = cv::norm(m_Center - frameCenter);


}

bool Armor::ArmorDetector(const cv::Mat& frame)
{
    if(std::max(m_LeftLight.Area(), m_RightLight.Area()) / std::min(m_LeftLight.Area(), m_RightLight.Area()) > aimbotConfig.lightAreaDiffRatio)
        return false;
    if (m_LeftLight.Ellipse().boundingRect2f().br().y < m_RightLight.Ellipse().boundingRect2f().tl().y || m_RightLight.Ellipse().boundingRect2f().br().y < m_LeftLight.Ellipse().boundingRect2f().tl().y)
        return false;
    if ((m_LeftLight.Area() + m_RightLight.Area()) / m_BBox.area() > 0.5)
        return false;
    if(m_AngleDiff > aimbotConfig.lightAngleDiff)
        return false;
    if(m_LenDiff > aimbotConfig.lightLenDiff)
    {
        return false;
    }
    float meanLen = (m_LeftLight.Length() + m_RightLight.Length()) / 2;                //灯条长度
    float LightDiffRatioX = fabs(m_LeftLight.Center().x - m_RightLight.Center().x) / meanLen;    //左右灯条中心点x的差值比例
    float LightDiffRatioY = fabs(m_LeftLight.Center().y - m_RightLight.Center().y) / meanLen;
    float ArmorAspectRatio = cv::norm(m_LeftLight.Ellipse().center - m_RightLight.Ellipse().center) / meanLen;

    // std::cout << m_LeftLight.Length() << "   "  << meanLen << "  LightDiffRatioY:" << LightDiffRatioY << "   LightDiffRatioX:" << LightDiffRatioX << "  ArmorAspectRatio:" << ArmorAspectRatio << std::endl;

    if(LightDiffRatioX > aimbotConfig.lightMinDiffRatioX
            && LightDiffRatioY < aimbotConfig.lightMaxDiffRatioY
            && ArmorAspectRatio > aimbotConfig.armorMinAspectRatio
            && ArmorAspectRatio < aimbotConfig.armorMaxAspectRatio)
    {

        if(ArmorAspectRatio > aimbotConfig.bigArmorRatio)
        {
            m_ArmorType = ArmorType::Big;
        }
        else
        {
            m_ArmorType = ArmorType::Small;
        }
#ifdef Yolo
        cv::Mat ArmorNum = frame(m_BBox);
        cv::resize(ArmorNum, ArmorNum, cv::Size(640, 640), 0, 0, cv::INTER_NEAREST);
        cv::cvtColor(ArmorNum, ArmorNum, CV_BGR2GRAY);
        cv::cvtColor(ArmorNum, ArmorNum, CV_GRAY2BGR);
        cv::dnn::blobFromImage(ArmorNum, ArmorNum, 1, cv::Size(128, 128), cv::Scalar(), true, false);

        time_t timeStart, timeEnd;
        timeStart = clock();
        m_Net.setInput(ArmorNum);
        cv::Mat Output = m_Net.forward();
        timeEnd = clock();
        // std::cout << Output << std::endl;
        std::cout << Output << std::endl << "推理时间：" << (timeEnd - timeStart) / 1000 << " ms" << std::endl;
#endif
        return true;
    }
    return false;
}

class Aimbot
{
private:
    RMColor m_TargetColor;
    Armor m_CurTargetArmor, m_LastTargetArmor;
    cv::Mat m_Frame;
    ros::Time m_Timestamp;
    bool m_DesignatedArmor = false;
    int m_DesignatedNum;
    int m_NoAmrorFrameNum;
    cv::Rect m_FrameROIRect;

public:
    Aimbot() {}
    Aimbot(int MyColor);
    ~Aimbot() {}
    bool ArmorDetect(Armor& targetArmor);
    void ColorSet(int MyColor);
    bool Process(FrameInfo& frameInfo, Armor& targetArmor);
    bool Process_NoTime(cv::Mat& frame, Armor& targetArmor);
};

Aimbot::Aimbot(int MyColor)
{
    if(MyColor == 1)
        m_TargetColor = RMColor::Blue;
    else
        m_TargetColor = RMColor::Red;
}

void Aimbot::ColorSet(int MyColor)
{
    if(MyColor == 1)
        m_TargetColor = RMColor::Blue;
    else
        m_TargetColor = RMColor::Red;
}

bool Aimbot::ArmorDetect(Armor& targetArmor)
{
    cv::Mat grayImg, binColorImg, brightnessImg, binBrightnessImg, binImg;
    std::vector<cv::Mat> channels;

    //亮度处理
    cv::cvtColor(m_Frame, brightnessImg, cv::COLOR_RGB2GRAY);
    cv::threshold(brightnessImg, binBrightnessImg, aimbotConfig.thresHoldBrightness, 255, cv::THRESH_BINARY);

    cv::Mat Element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    //颜色处理
    cv::split(m_Frame,channels);    //分割
    if(m_TargetColor==RMColor::Red)
        grayImg=channels.at(2)-channels.at(0);//Get red-blue image;
    else
        grayImg=channels.at(0)-channels.at(2);//Get blue-red image;


    threshold(grayImg, binColorImg, aimbotConfig.thresHoldColor, 255, cv::THRESH_BINARY);   //二值化

    //二合一
    cv::bitwise_and(binBrightnessImg, binColorImg, binImg);

    cv::dilate(binImg, binImg, Element);   //膨胀


    std::vector<std::vector<cv::Point>> lightContours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<Light> lightInfos;
    std::vector<Armor> armorInfos;

    cv::findContours(binImg, lightContours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);   //函数详见https://blog.csdn.net/guduruyu/article/details/69220296
    for(int i = 0; i < lightContours.size(); i++)
    {
        if(lightContours[i].size() >= 6  && hierarchy[i][3] == -1)
        {
            Light possLight(lightContours[i], m_FrameROIRect);
            if(possLight.LightDetector())
            {
                lightInfos.push_back(possLight);
            }
        }
    }
    std::sort(lightInfos.begin(), lightInfos.end(), [](const Light& ld1, const Light& ld2)
    {
        return ld1.Center().x < ld2.Center().x;
    });
    for (int i = 0; i < (int)lightInfos.size() - 1 ; i++)
    {
        for (int j = i+1; j < lightInfos.size(); j++)
        {
            Armor possArmor(lightInfos[i], lightInfos[j]);
            bool isAddArmor = true;

            if(possArmor.ArmorDetector(m_Frame))
            {
                for (int l = i+1; l < lightInfos.size(); l++)
                {
                    if(cv::pointPolygonTest(possArmor.Vertexes(), lightInfos[l].Center(), false) >= 0)
                    {
                        isAddArmor = false;
                        break;
                    }
                }
                if(!isAddArmor)
                {
                    if(!m_DesignatedArmor)
                        armorInfos.push_back(possArmor);
                    else if(possArmor.Id() == m_DesignatedNum)
                        armorInfos.push_back(possArmor);
                }

            }
        }

    }

    bool isFoundArmor = true;
    static bool isFoundArmorLast = false;

    if(armorInfos.empty())
    {
        isFoundArmor = false;
    }
    else
    {

        double minDist =  std::numeric_limits<double>::max();   //函数详见https://blog.csdn.net/fengbingchun/article/details/77922558   新老装甲板最小距离
        if(isFoundArmorLast)
        {
            for(const auto& targetArmor : armorInfos)
            {
                double diffDist = cv::norm(m_LastTargetArmor.Center() - targetArmor.Center());
                if (diffDist < minDist)
                {
                    minDist = diffDist;
                    m_CurTargetArmor = targetArmor;
                }
            }
        }
        else
        {
            std::sort(armorInfos.begin(), armorInfos.end(), [](const Armor& ad1, const Armor& ad2)
            {
                return ad1.Target2CenterDist() < ad2.Target2CenterDist();
            });
            m_CurTargetArmor = armorInfos[0];
        }

        isFoundArmorLast = isFoundArmor;

        if (isFoundArmor)
        {
            m_NoAmrorFrameNum = 0;
            m_LastTargetArmor = m_CurTargetArmor;
        }
        else
        {
            if (m_NoAmrorFrameNum > 300)
            {
                m_LastTargetArmor = m_CurTargetArmor;
            }
        }

        if (isFoundArmor)
        {
            targetArmor = m_CurTargetArmor;
            m_FrameROIRect = setROIRect(m_CurTargetArmor.BBox(), 2, 2);
            m_NoAmrorFrameNum = 0;
        }
        else
        {
            m_NoAmrorFrameNum++;
            double roiSize = 0.8 * m_NoAmrorFrameNum;
            if (roiSize < 1)
                roiSize = 1;
            m_FrameROIRect = setROIRect(m_FrameROIRect, roiSize, roiSize);
        }

        m_FrameROIRect = fullROIRect;

    }

    return isFoundArmor;


}

bool Aimbot::Process(FrameInfo& frameInfo, Armor& targetArmor)
{
    frameInfo.frame.copyTo(m_Frame);
    m_Timestamp = frameInfo.timestamp;
    time_t start, end;
    bool FoundArm = false;

    double yaw_measured = 0, pitch_measured = 0, dist = 0;
    start = clock();
    if (ArmorDetect(targetArmor))
    {
        FoundArm = true;
    }
    end = clock();
#ifdef DEBUG
    std::cout<<"总时间: "<<(end-start)/1000<<" ms"<<std::endl;
#endif
    return FoundArm;

}

bool Aimbot::Process_NoTime(cv::Mat& frame, Armor& targetArmor)
{
    frame.copyTo(m_Frame);
    time_t start, end;

    bool FoundArm = false;

    double yaw_measured = 0, pitch_measured = 0, dist = 0;
    start = clock();
    if (ArmorDetect(targetArmor))
    {
        FoundArm = true;
    }
    end = clock();
#ifdef DEBUG
    std::cout<<"总时间: "<<(end-start)/1000<<" ms"<<std::endl;
#endif

    return FoundArm;
}

