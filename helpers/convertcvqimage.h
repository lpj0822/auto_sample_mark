#ifndef CONVERTCVQIMAGE_H
#define CONVERTCVQIMAGE_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <QImage>
#include <QRgb>

class ConvertCVQImage
{
public:
    ConvertCVQImage();
    ~ConvertCVQImage();

    QImage cvMatToQImage(const cv::Mat& mat);
    cv::Mat QImageTocvMat(QImage& image);
};

#endif // CONVERTCVQIMAGE_H
