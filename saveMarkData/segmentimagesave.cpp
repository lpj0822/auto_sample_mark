#include "segmentimagesave.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "sampleMarkParam/manualparamterconfig.h"

SegmentImageSave::SegmentImageSave(QObject *parent) : QObject(parent)
{

}

SegmentImageSave::~SegmentImageSave()
{

}

int SegmentImageSave::createSegmentImage()
{
    return 0;
}

int SegmentImageSave::readSegmentImage(const QString &imageFilePath, MyObject &object)
{
    QImage segmentImage;
    if(segmentImage.load(imageFilePath))
    {
        object.setSegmentImage(segmentImage);
        return 0;
    }
    else
    {
        return -1;
    }
}

QImage SegmentImageSave::generateMaskFromPolygon(const QList<MyObject> &obejcts, const int width, const int height)
{
    QImage result;
    cv::Mat src = cv::Mat::zeros(height, width, CV_8UC3);
    for(int loop = 0; loop < obejcts.count(); loop++)
    {
        const MyObject object = obejcts[loop];
        const QPolygon polygon = object.getPolygon();
        const QString className = object.getObjectClass();
        QString colorStr = ManualParamterConfig::getMarkClassColor(className);
        QColor color(colorStr);
        std::vector<cv::Point > contour;
        for(int pointIndex = 0; pointIndex < polygon.count(); pointIndex++)
        {
            contour.push_back(cv::Point(polygon[pointIndex].x(), polygon[pointIndex].y()));
        }
        cv::polylines(src, contour, true, cv::Scalar(color.red(), color.green(), color.blue()), 2, cv::LINE_AA);
        std::vector<std::vector<cv::Point >> contours;
        contours.push_back(contour);
        cv::fillPoly(src, contours, cv::Scalar(color.red(), color.green(), color.blue()));
    }
    result = QImage((uchar*)src.data, src.cols, src.rows, QImage::Format_RGB888);
    return result;
}
