#include "ssdector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iomanip>
#include <iostream>

SSDector::SSDector()
{
    initData();
}

SSDector::~SSDector()
{

}

int SSDector::initModel(const std::string caffeNet, const std::string caffeModel)
{
    //Import Caffe SSD model
    try
    {
        net = cv::dnn::readNetFromCaffe(caffeNet, caffeModel);
    }
    catch (const cv::Exception &err) //Importer can throw errors, we will catch them
    {
        std::cout << err.msg << std::endl;
        return -1;
    }
    return 0;
}

int SSDector::initDetectorParameters(const int dataWidth, const int dataHeight,
                                     const float confidenceThreshold, const QMap<int, QString> &labels)
{
    this->inputDataWidth = dataWidth;
    this->inputDataHeight = dataHeight;
    this->confidenceThreshold = confidenceThreshold;
    this->labelIds.clear();
    this->labelNames.clear();
    for(QMap<int, QString>::const_iterator iterator = labels.constBegin(); iterator != labels.constEnd(); ++iterator)
    {
        this->labelIds.push_back(iterator.key());
        this->labelNames.push_back(iterator.value().toStdString());
    }
    return 0;
}

void SSDector::processDetect(const cv::Mat &inputImage, std::vector<cv::Rect> &objectRect,
                               std::vector<std::string> &objectClass, std::vector<float> &objectConfidence)
{
    if(inputImage.empty())
    {
        return;
    }

    cv::Mat detectionObject;
    int width = inputImage.cols;
    int height = inputImage.rows;
    double duration = static_cast<double>(cv::getTickCount());
    objectRect.clear();
    if(width <= (this->inputDataWidth * 1.9) && height <= (this->inputDataHeight * 1.9))
    {
        detectionObject = detect(inputImage);
        processDetectionObject(inputImage, detectionObject, 0, 0, objectRect, objectClass, objectConfidence);
    }
    else if(width <= (this->inputDataWidth * 1.9))
    {
        int topX = 0;
        int topY = height / 2;
        const cv::Mat roi = inputImage(cv::Rect(topX, topY, width - topX, height - topY));
        detectionObject = detect(roi);
        processDetectionObject(roi, detectionObject, topX, topY, objectRect, objectClass, objectConfidence);
    }
    else if(height <= (this->inputDataHeight * 1.9))
    {
        int topX1 = 0;
        int topY1 = 0;
        int topX2 = width / 2;
        int topY2 = 0;
        const cv::Mat roi1 = inputImage(cv::Rect(topX1, topY1, topX2 - topX1, height - topY1));
        detectionObject = detect(roi1);
        processDetectionObject(roi1, detectionObject, topX1, topY1, objectRect, objectClass, objectConfidence);
        const cv::Mat roi2 = inputImage(cv::Rect(topX2, topY2, width - topX2, height - topY2));
        detectionObject = detect(roi2);
        processDetectionObject(roi2, detectionObject, topX2, topY2, objectRect, objectClass, objectConfidence);
    }
    else
    {
        int topX1 = 0;
        int topY1 = height / 2;
        int topX2 = width / 2;
        int topY2 = height / 2;
        const cv::Mat roi1 = inputImage(cv::Rect(topX1, topY1, topX2 - topX1, height - topY1));
        detectionObject = detect(roi1);
        processDetectionObject(roi1, detectionObject, topX1, topY1, objectRect, objectClass, objectConfidence);
        const cv::Mat roi2 = inputImage(cv::Rect(topX2, topY2, width - topX2, height - topY2));
        detectionObject = detect(roi2);
        processDetectionObject(roi2, detectionObject, topX2, topY2, objectRect, objectClass, objectConfidence);
    }

    duration = (static_cast<double>(cv::getTickCount()) - duration) / cv::getTickFrequency();
    std::cout << "time(sec):" << std::fixed << std::setprecision(4) << duration << std::endl;
//    showDetection(const_cast<cv::Mat&>(image), objectRect, objectClass, objectConfidence);
//    cv::imshow("SSD", image);
//    cv::waitKey(20);
}

cv::Mat SSDector::detect(const cv::Mat &image)
{
    if(image.empty())
    {
        return cv::Mat();
    }
    //Convert Mat to batch of images
    cv::Mat inputBlob = cv::dnn::blobFromImage(image, 1.0f, cv::Size(this->inputDataWidth, this->inputDataHeight),
                                               cv::Scalar(104, 117, 123), false, false);

    net.setInput(inputBlob, "data"); //set the network input
    cv::Mat detection = net.forward("detection_out");
    return detection;
}

void SSDector::processDetectionObject(const cv::Mat& roi, const cv::Mat& detectionObjects, const int topX, const int topY,
                                      std::vector<cv::Rect> &objectRect, std::vector<std::string> &objectClass,
                                      std::vector<float> &objectConfidence)
{
    if(roi.empty())
    {
        return;
    }

    for(int i = 0; i < detectionObjects.rows; i++)
    {
        float confidence = detectionObjects.at<float>(i, 2);

        if(confidence > confidenceThreshold)
        {
            int classIndex = static_cast<int>(detectionObjects.at<float>(i, 1));
            int xLeftBottom = static_cast<int>(detectionObjects.at<float>(i, 3) * roi.cols) + topX;
            int yLeftBottom = static_cast<int>(detectionObjects.at<float>(i, 4) * roi.rows) + topY;
            int xRightTop = static_cast<int>(detectionObjects.at<float>(i, 5) * roi.cols) + topX;
            int yRightTop = static_cast<int>(detectionObjects.at<float>(i, 6) * roi.rows) + topY;

            objectRect.push_back(cv::Rect(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom,
                                          yRightTop - yLeftBottom));

            std::string objectName = getLabelName(classIndex);
            objectClass.push_back(objectName);

            objectConfidence.push_back(confidence);

//            std::cout << "Class: " << objectName << std::endl;
//            std::cout << "Confidence: " << confidence << std::endl;
//            std::cout << " " << xLeftBottom
//                             << " " << yLeftBottom
//                             << " " << xRightTop
//                             << " " << yRightTop << std::endl;
        }
    }
}

std::string SSDector::getLabelName(const int indice)
{
    std::string labelName = "";
    for(size_t index = 0; index < labelIds.size(); index++)
    {
        if(labelIds[index] == indice)
        {
            labelName = labelNames[index];
            break;
        }
    }
    return labelName;
}

void SSDector::showDetection(cv::Mat &image, std::vector<cv::Rect> &objectRect,
                   std::vector<std::string> &objectClass, std::vector<float> &objectConfidence)
{
    if(image.empty())
    {
        return;
    }

    int objectColor = 10;
    for(size_t index = 0; index < objectRect.size(); index++)
    {
        cv::rectangle(image, objectRect[index],
                      cv::Scalar(255 * (objectColor / 100), 255 * ((objectColor / 10) % 10), 255 * (objectColor % 10)));
        cv::putText(image, objectClass[index], objectRect[index].tl(), cv::FONT_HERSHEY_COMPLEX, 0.5,
                    cv::Scalar(255 * (objectColor / 100), 255 * ((objectColor / 10) % 10), 255 * (objectColor % 10)));
    }
}

void SSDector::initData()
{
    this->inputDataWidth = 512;
    this->inputDataHeight = 512;
    this->confidenceThreshold = 0.5f;

    this->labelIds.clear();
    this->labelNames.clear();
}

void SSDector::saveConfig()
{

}

void SSDector::loadConfig()
{

}
