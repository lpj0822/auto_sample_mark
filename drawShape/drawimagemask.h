#ifndef DRAWIMAGEMASK_H
#define DRAWIMAGEMASK_H

#include <QObject>
#include <QList>
#include <QImage>
#include <QPoint>
#include "dataType/myobject.h"
#include "saveMarkData/segmentationmaskprocess.h"

class DrawImageMask : public QObject
{
    Q_OBJECT
public:
    DrawImageMask(bool isCurveFit=false, QObject *parent = nullptr);
    ~DrawImageMask();

    void drawLaneMaskImage(const MyObject &object, const QString &visibleSampleClass, QImage &maskImage);
    void drawPolygonMaskImage(const MyObject &object, const QString &visibleSampleClass, QImage &maskImage);

signals:

public slots:

private:
    void drawLaneMask(const QList<QPoint> &pointList, const int width,
                      const QColor &color, QImage &maskImage);
    void drawPolygonMask(const QPolygon &drawPolygon, const QColor &color, QImage &maskImage);

    void initDraw();

private:
    bool isCurveFit;
    SegmentationMaskProcess maskProcess;
};

#endif // DRAWIMAGEMASK_H
