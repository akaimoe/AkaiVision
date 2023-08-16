#include "bits/stdc++.h"
#include <fstream>
#include <sstream>
#include "opencv2/opencv.hpp"
#include <onnxruntime_cxx_api.h>


//#define VISUALIZE

// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT = cv::FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;

// Colors.
cv::Scalar BLACK(0, 0, 0);
cv::Scalar BLUE(255, 178, 50);
cv::Scalar YELLOW(0, 255, 255);
cv::Scalar RED(0, 0, 255);
cv::Scalar GREEN(0, 255, 128);
cv::Scalar ORANGE(0, 97, 255);
std::array<cv::Scalar, 5> kLandmarkColors{ BLUE, GREEN, RED, ORANGE, YELLOW };

void draw_label(cv::Mat& input_image, std::string label, cv::Point topLeft)
{
    int left = topLeft.x;
    int top = topLeft.y;
    // Display the label at the top of the bounding box.
    int baseLine;
    cv::Size label_size = cv::getTextSize(label, FONT, FONT_SCALE, THICKNESS, &baseLine);
    top = std::max(top, label_size.height);
    // Top left corner.
    cv::Point tlc = cv::Point(left, top - 30);
    // Bottom right corner.
    cv::Point brc = cv::Point(left + label_size.width, top + label_size.height + baseLine - 30);
    // Draw black rectangle.
    cv::rectangle(input_image, tlc, brc, BLACK, cv::FILLED);
    // Put the label on the black rectangle.
    cv::putText(input_image, label, cv::Point(left, top + label_size.height - 30), FONT, FONT_SCALE, YELLOW, THICKNESS);
}

class DetId
{
public:
    DetId() = default;
    DetId(int clsId) : m_Id(DetClass(clsId)) {}
    int Int() const
    {
        return static_cast<int>(m_Id);
    }
    bool IsRedArmor() const
    {
        return static_cast<int>(DetClass::RS) <= static_cast<int>(m_Id)
               && static_cast<int>(m_Id) <= static_cast<int>(DetClass::RBa)
               && static_cast<int>(m_Id) != static_cast<int>(DetClass::RBs);
    }
    bool IsBlueArmor() const
    {
        return static_cast<int>(DetClass::BS) <= static_cast<int>(m_Id)
               && static_cast<int>(m_Id) <= static_cast<int>(DetClass::BBa)
               && static_cast<int>(m_Id) != static_cast<int>(DetClass::BBs);
    }
    bool IsGreyArmor() const
    {
        return static_cast<int>(DetClass::NS) <= static_cast<int>(m_Id)
               && static_cast<int>(m_Id) <= static_cast<int>(DetClass::NBa)
               && static_cast<int>(m_Id) != static_cast<int>(DetClass::NBs);
    }
    bool IsArmor() const
    {
        return IsRedArmor() || IsBlueArmor() || IsGreyArmor();
    }

    // 英雄1，工程2，步兵345，哨兵6，前哨站7，基地8
    int ArmorDigit() const {
        if (!IsRedArmor())
            return 0;

        return m_ArmorDigitMapping.at(static_cast<int>(m_Id));
    }

private:
    enum class DetClass {
        BS = 0,  // 哨兵
        B1,
        B2,
        B3,
        B4,
        B5,
        BO,      // 前哨站
        BBs,     // 基地（本身）
        BBa,     // 基地大装甲
        RS,
        R1,
        R2,
        R3,
        R4,
        R5,
        RO,
        RBs,
        RBa,
        NS,
        N1,
        N2,
        N3,
        N4,
        N5,
        NO,
        NBs,
        NBa,
        PS,
        P1,
        P2,
        P3,
        P4,
        P5,
        PO,
        PBs,
        PBa
    } m_Id;

    std::unordered_map<int, int> m_ArmorDigitMapping = {
        {0, 6}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 7}, {8, 8},     // 蓝
        {9, 6}, {10,1}, {11,2}, {12,3}, {13,4}, {14,5}, {15,7}, {17,8},     // 红
        {18,6}, {19,1}, {20,2}, {21,3}, {22,4}, {23,5}, {24,7}, {26,8},     // 熄灭
        {27,6}, {28,1}, {29,2}, {30,3}, {31,4}, {32,5}, {33,7}, {35,8},     // 紫色
    };
} UnknownColorId(10);




struct NetConfig
{
    float confThreshold; // Confidence threshold
    float nmsThreshold;  // Non-maximum suppression threshold
    std::string modelpath;
    std::string classListPath;
};


class YoloKpts
{
public:
    YoloKpts(const NetConfig& config);
    cv::Mat detect(const cv::Mat& frame);
private:
    int inpWidth;
    int inpHeight;
    std::vector<std::string> class_list;
    int nout;
    int num_proposal;

    float confThreshold;
    float nmsThreshold;
    bool has_postprocess;

    Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "YoloKpts");
    Ort::Session* ort_session = nullptr;
    Ort::SessionOptions sessionOptions = Ort::SessionOptions();
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<std::vector<int64_t>> input_node_dims; // >=1 outputs
    std::vector<std::vector<int64_t>> output_node_dims; // >=1 outputs
};

YoloKpts::YoloKpts(const NetConfig& config)
{
    this->confThreshold = config.confThreshold;
    this->nmsThreshold = config.nmsThreshold;
#ifdef _WIN32
    std::wstring model_path = std::wstring(config.modelpath.begin(), config.modelpath.end());
#else  // linux
    std::string model_path = config.modelpath;
#endif // _WIN32
    //OrtStatus* status = OrtSessionOptionsAppendExecutionProvider_CUDA(sessionOptions, 0);
    sessionOptions.SetIntraOpNumThreads(4);
    sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    ort_session = new Ort::Session(env, model_path.c_str(), sessionOptions);
    size_t numInputNodes = ort_session->GetInputCount();
    size_t numOutputNodes = ort_session->GetOutputCount();
    Ort::AllocatorWithDefaultOptions allocator;

    for (int i = 0; i < numInputNodes; i++) {
        input_names.push_back(std::string(""));
    }
    for (int i = 0; i < numOutputNodes; i++) {
        output_names.push_back(std::string(""));
    }
    for (int i = 0; i < numInputNodes; i++)
    {
        auto p = ort_session->GetInputNameAllocated(i, allocator);
        input_names[i].append(p.get());
        Ort::TypeInfo input_type_info = ort_session->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_dims = input_tensor_info.GetShape();
        input_node_dims.push_back(input_dims);
    }
    for (int i = 0; i < numOutputNodes; i++)
    {
        auto p = ort_session->GetOutputNameAllocated(i, allocator);
        output_names[i].append(p.get());
        Ort::TypeInfo output_type_info = ort_session->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto output_dims = output_tensor_info.GetShape();
        output_node_dims.push_back(output_dims);
    }
    this->inpHeight = input_node_dims[0][2];
    this->inpWidth = input_node_dims[0][3];

    std::ifstream ifs(config.classListPath);
    std::string line;

    while (std::getline(ifs, line))
    {
        class_list.push_back(line);
    }
}

cv::Mat YoloKpts::detect(const cv::Mat& frame)
{
    static cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1. / 255., cv::Size(this->inpWidth, this->inpHeight), cv::Scalar(), true, false);
    static const std::array<int64_t, 4> input_shape_{ 1, 3, this->inpHeight, this->inpWidth };

    static Ort::MemoryInfo allocator_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Value input_tensor_ = Ort::Value::CreateTensor<float>(allocator_info, blob.ptr<float>(), blob.total(), input_shape_.data(), input_shape_.size());

    // 开始推理
    static const std::array inputNames = { input_names[0].c_str() };
    static const std::array outNames = { output_names[0].c_str()};
    std::vector<Ort::Value> ort_outputs = ort_session->Run(Ort::RunOptions{ nullptr }, inputNames.data(), &input_tensor_, 1, outNames.data(), output_names.size());   // 开始推理

    Ort::Value& predictions = ort_outputs.at(0);
    auto pred_dims = predictions.GetTensorTypeAndShapeInfo().GetShape();
    num_proposal = pred_dims.at(1);
    nout = pred_dims.at(2);

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> allBoxes;
    std::vector<std::array<float, 10>> allLandmarks;
    float ratioh = (float)frame.rows / this->inpHeight, ratiow = (float)frame.cols / this->inpWidth;
    float* pdata = predictions.GetTensorMutableData<float>();
    for (int n = 0; n < this->num_proposal; n++)   ///特征图尺度
    {
        float box_score = pdata[4];
        if (box_score > this->confThreshold)
        {
            float* DetId_scores = pdata + 15;
            cv::Mat scores(1, class_list.size(), CV_32FC1, DetId_scores);
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            if (max_class_score > this->confThreshold)
            {
                float cx = pdata[0] * ratiow;  ///cx
                float cy = pdata[1] * ratioh;   ///cy
                float w = pdata[2] * ratiow;   ///w
                float h = pdata[3] * ratioh;  ///h

                float xmin = cx - 0.5 * w;
                float ymin = cy - 0.5 * h;
                float xmax = cx + 0.5 * w;
                float ymax = cy + 0.5 * h;

                confidences.push_back(box_score);
                class_ids.push_back(class_id.x);
                allLandmarks.push_back(std::array<float, 10> {pdata[5] * ratiow, pdata[6] * ratioh, pdata[7] * ratiow, pdata[8] * ratioh, pdata[9] * ratiow, pdata[10] * ratioh, pdata[11] * ratiow, pdata[12] * ratioh, pdata[13] * ratiow, pdata[14] * ratioh});
                allBoxes.push_back(cv::Rect(xmin, ymin, w, h));
            }
        }
        pdata += nout;
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    //nms(generate_boxes);
    static std::vector<int> indices;
    cv::dnn::NMSBoxes(allBoxes, confidences, confThreshold, nmsThreshold, indices);


#ifdef VISUALIZE
    cv::Mat frame_to_draw = frame.clone();
    for (int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        cv::Rect box = allBoxes[idx];
        std::array<float, 10> landmarks = allLandmarks[idx];
        DetId detId(class_ids[idx]);
        //cars.emplace_back(box, detId, confidences[idx]);

        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;

        // Draw bounding box.
        rectangle(frame_to_draw, cv::Point(left, top), cv::Point(left + width, top + height), BLUE, 1);

        // Draw landmarks
        for (int j = 0; j < 5; ++j)
        {
            cv::circle(frame_to_draw, cv::Point(landmarks[2 * j], landmarks[2 * j + 1]), 3, kLandmarkColors[j], -1);
        }


        // Get the label for the class name and its confidence.
        std::string label = cv::format("%.2f", confidences[idx]);
        label = class_list[class_ids[idx]] + ":" + label;

        // Draw class labels.
        draw_label(frame_to_draw, label, cv::Point(left, top));
    }

    return frame_to_draw;
#endif // VISUALIZE

    return frame;

}


int main(int argc, char  *argv[])
{
    NetConfig YoloKpts_cfg = { 0.45, 0.45, "../best-sim.onnx", "../cls.txt" };
    YoloKpts net(YoloKpts_cfg);
    cv::VideoCapture cap("/home/altair/test/video.avi");
    cv::Mat srcimg;
    static const std::string kWinName = "RM Armor DL in ONNXRuntime";
    //namedWindow(kWinName, WINDOW_NORMAL);
    while (true)
    {
        if (!cap.read(srcimg))
            break;
        auto t1 = std::chrono::high_resolution_clock::now();
        cv::Mat dst = net.detect(srcimg);
        auto t2 = std::chrono::high_resolution_clock::now();

        auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        std::string label = cv::format("time : %ld ms", time_cost);
        std::cout << label << std::endl;

#ifdef VISUALIZE
        cv::imshow(kWinName, dst);
        cv::waitKey(1);
#endif // VISUALIZE


    }

    return 0;
}