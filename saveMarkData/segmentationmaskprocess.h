#ifndef SEGMENTATIONMASKPROCESS_H
#define SEGMENTATIONMASKPROCESS_H

#include <QObject>
#include <QImage>
#include <QColor>
#include <QPoint>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "dataType/myobject.h"
#include "baseAlgorithm/curvealgorithm.h"

class SegmentationMaskProcess : public QObject
{
    Q_OBJECT
public:
    SegmentationMaskProcess(QObject *parent = nullptr);
    ~SegmentationMaskProcess();

    QImage createSegmentMask(const QList<MyObject> &obejcts, const int width, const int height);
    int readSegmentMask(const QString &imageFilePath, MyObject &object);

    void generateMaskFromPolygon(const MyObject &obejcts, cv::Mat &mask);
    void generateMaskFromLane(const MyObject &obejcts, cv::Mat &mask);

    void getCurveFitPointList(const QList<QPoint> &inputKeyPoint, QList<QPoint> &result);

signals:

public slots:

private:
    CurveAlgorithm curveFit;
};

#endif // SEGMENTATIONMASKPROCESS_H
