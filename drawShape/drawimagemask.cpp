#include "drawimagemask.h"
#include "sampleMarkParam/manualparamterconfig.h"
#include <QPainter>
#include <QPen>
#include <QBrush>

DrawImageMask::DrawImageMask(bool isCurveFit, QObject *parent) : QObject(parent), isCurveFit(isCurveFit)
{
    initDraw();
}

DrawImageMask::~DrawImageMask()
{

}

void DrawImageMask::drawLaneMaskImage(const MyObject &object, const QString &visibleSampleClass, QImage &maskImage)
{
    int laneWidth = 1;
    QList<QPoint> resultPoints;
    QString color = ManualParamterConfig::getMarkClassColor(object.getObjectClass());
    QColor drawColor(color);
    laneWidth = object.getLineWidth();
    if(!drawColor.isValid())
    {
        drawColor = QColor("#000000");
    }
    if(visibleSampleClass == "All")
    {
        if(isCurveFit)
        {
            maskProcess.getCurveFitPointList(object.getPointList(), resultPoints);
        }
        else
        {
            resultPoints = object.getPointList();
        }
        drawLaneMask(resultPoints, laneWidth, drawColor, maskImage);
    }
    else
    {
        if(object.getObjectClass().contains(visibleSampleClass))
        {
            if(isCurveFit)
            {
                maskProcess.getCurveFitPointList(object.getPointList(), resultPoints);
            }
            else
            {
                resultPoints = object.getPointList();
            }
            drawLaneMask(resultPoints, laneWidth, drawColor, maskImage);
        }
    }
}

void DrawImageMask::drawPolygonMaskImage(const MyObject &obejct, const QString &visibleSampleClass, QImage &maskImage)
{
    QString color = ManualParamterConfig::getMarkClassColor(obejct.getObjectClass());
    QColor drawColor(color);
    if(!drawColor.isValid())
    {
        drawColor = QColor("#000000");
    }
    if(visibleSampleClass == "All")
    {
        drawPolygonMask(obejct.getPolygon(), drawColor, maskImage);
    }
    else
    {
        if(obejct.getObjectClass().contains(visibleSampleClass))
        {
            drawPolygonMask(obejct.getPolygon(), drawColor, maskImage);
        }
    }
}

void DrawImageMask::drawLaneMask(const QList<QPoint> &pointList, const int width,
                                 const QColor &color, QImage &maskImage)
{
    if(!maskImage.isNull())
    {
        int harf_width = width / 2;
        QPainter painter;
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setRenderHint(QPainter::HighQualityAntialiasing);
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        QPen pen(color, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QBrush brush(color, Qt::SolidPattern);
        painter.begin(&maskImage);
        painter.setPen(pen);
        painter.setBrush(brush);
        for(int loop = 0; loop < pointList.count(); loop++)
        {
            int y = std::min(pointList[loop].y(), maskImage.height());
            int minX = std::max(0, pointList[loop].x() - harf_width);
            int maxX = std::min(pointList[loop].x() + harf_width, maskImage.width());
            painter.drawLine(QPoint(minX, y), QPoint(maxX, y));
        }
        painter.end();
    }
}

void DrawImageMask::drawPolygonMask(const QPolygon &drawPolygon, const QColor &color, QImage &maskImage)
{
    if(!maskImage.isNull())
    {
        QPainter painter;
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setRenderHint(QPainter::HighQualityAntialiasing);
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        QPen pen(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QBrush brush(color, Qt::SolidPattern);
        painter.begin(&maskImage);
        painter.setPen(pen);
        painter.setBrush(brush);
        painter.drawPolygon(drawPolygon);
        painter.end();
    }
}

void DrawImageMask::initDraw()
{

}
