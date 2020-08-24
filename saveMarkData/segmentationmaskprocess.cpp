#include "segmentationmaskprocess.h"
#include "sampleMarkParam/manualparamterconfig.h"

SegmentationMaskProcess::SegmentationMaskProcess(QObject *parent) : QObject(parent)
{

}

SegmentationMaskProcess::~SegmentationMaskProcess()
{

}

QImage SegmentationMaskProcess::createSegmentMask(const QList<MyObject> &obejcts, const int width, const int height)
{
    QImage result;
    cv::Mat rgbMat;
    cv::Mat src = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for(int loop = 0; loop < obejcts.count(); loop++)
    {
        const MyObject object = obejcts[loop];
        if(object.getShapeType() == ShapeType::SEGMENT_POLYGON_SHAPE)
        {
            generateMaskFromPolygon(object, src);
        }
        else if(object.getShapeType() == ShapeType::POLYLINE_SHAPE)
        {
            generateMaskFromLane(object, src);
        }
    }
    cv::cvtColor(src, rgbMat, cv::COLOR_BGR2RGB);
    result = QImage((uchar*)rgbMat.data, rgbMat.cols, rgbMat.rows, QImage::Format_RGB888).copy();
    return result;
}

int SegmentationMaskProcess::readSegmentMask(const QString &imageFilePath, MyObject &object)
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

void SegmentationMaskProcess::generateMaskFromPolygon(const MyObject &object, cv::Mat &mask)
{
    if(mask.empty())
        return;
    const QPolygon polygon = object.getPolygon();
    const QString className = object.getObjectClass();
    QString colorStr = ManualParamterConfig::getMarkClassColor(className);
    QColor color(colorStr);
    std::vector<cv::Point > contour;
    if(!color.isValid())
    {
        color = QColor("#000000");
    }
    for(int pointIndex = 0; pointIndex < polygon.count(); pointIndex++)
    {
        contour.push_back(cv::Point(polygon[pointIndex].x(), polygon[pointIndex].y()));
    }
    cv::polylines(mask, contour, true, cv::Scalar(color.red(), color.green(), color.blue()), 2, cv::LINE_AA);
    std::vector<std::vector<cv::Point >> contours;
    contours.push_back(contour);
    cv::fillPoly(mask, contours, cv::Scalar(color.red(), color.green(), color.blue()));
}

void SegmentationMaskProcess::generateMaskFromLane(const MyObject &object, cv::Mat &mask)
{
    if(mask.empty())
        return;
    const QString className = object.getObjectClass();
    const int laneWidth = object.getLineWidth();
    const int width = mask.cols;
    const int height = mask.rows;
    QString colorStr = ManualParamterConfig::getMarkClassColor(className);
    QColor color(colorStr);
    int harf_width = laneWidth / 2;
    QList<QPoint> resultPoints;
    resultPoints.clear();
    if(!color.isValid())
    {
        color = QColor("#000000");
    }
    getCurveFitPointList(object.getPointList(), resultPoints);
    for(int loop = 0; loop < resultPoints.count(); loop++)
    {
        int y = std::min(resultPoints[loop].y(), height);
        int minX = std::max(0, resultPoints[loop].x() - harf_width);
        int maxX = std::min(resultPoints[loop].x() + harf_width, width);
        cv::line(mask, cv::Point(minX, y), cv::Point(maxX, y), cv::Scalar(color.red(), color.green(), color.blue()));
    }
}

void SegmentationMaskProcess::getCurveFitPointList(const QList<QPoint> &inputKeyPoint, QList<QPoint> &result)
{
    cv::Mat A;
    int minX = 1e7;
    int maxX = 0;
    result.clear();
    if(inputKeyPoint.size() > 0)
    {
        curveFit.polynomialCurveFit(inputKeyPoint, 3, A);
        for(int loop = 0; loop < inputKeyPoint.count(); loop++)
        {
            if(inputKeyPoint[loop].x() > maxX)
                maxX = inputKeyPoint[loop].x();
            if(inputKeyPoint[loop].x() < minX)
                minX = inputKeyPoint[loop].x();
        }
        for (double x = minX; x <= maxX; x += 0.05)
        {
            double y = A.at<double>(0, 0) + A.at<double>(1, 0) * x +
                    A.at<double>(2, 0)*std::pow(x, 2) + A.at<double>(3, 0)*std::pow(x, 3);
            int tempX = static_cast<int>(std::round(x));
            int tempY = static_cast<int>(std::round(y));
            result.append(QPoint(tempX, tempY));
        }
    }
}
