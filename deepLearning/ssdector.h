#ifndef SSDECTOR_H
#define SSDECTOR_H

#include "caffedetector.h"
#include <QMap>

class SSDector : public CaffeDetector
{
public:
    SSDector();
    ~SSDector();

    int initModel(const std::string caffeNet, const std::string caffeModel);

    int initDetectorParameters(const int dataWidth, const int dataHeight,
                               const float confidenceThreshold, const QMap<int, QString> &labels);

    void processDetect(const cv::Mat &inputImage, std::vector<cv::Rect> &objectRect,
                                   std::vector<std::string> &objectClass, std::vector<float> &objectConfidence);

private:

    cv::Mat detect(const cv::Mat &image);

    void processDetectionObject(const cv::Mat& roi, const cv::Mat& detectionObjects, const int topX, const int topY,
                                std::vector<cv::Rect> &objectRect, std::vector<std::string> &objectClass,
                                std::vector<float> &objectConfidence);

    std::string getLabelName(const int indice);

    void showDetection(cv::Mat &image, std::vector<cv::Rect> &objectRect,
                       std::vector<std::string> &objectClass, std::vector<float> &objectConfidence);

    void initData();

private:

    //Initialize network
    cv::dnn::Net net;

    int inputDataWidth;
    int inputDataHeight;
    float confidenceThreshold;
    std::vector<int> labelIds;
    std::vector<std::string> labelNames;

private:

     void saveConfig();
     void loadConfig();

};

#endif // SSDECTOR_H
