#ifndef CAFFEDETECTOR_H
#define CAFFEDETECTOR_H
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>

class CaffeDetector
{
public:
    CaffeDetector();
    virtual ~CaffeDetector();

    virtual int initModel(const std::string caffeNet, const std::string caffeModel) = 0;
    virtual void processDetect(const cv::Mat &inputImage, std::vector<cv::Rect> &objectRect,
                               std::vector<std::string> &objectClass, std::vector<float> &objectConfidence) = 0;

private:
  virtual void saveConfig() = 0;
  virtual void loadConfig() = 0;
};

#endif // CAFFEDETECTOR_H
