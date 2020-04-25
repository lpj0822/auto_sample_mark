#ifndef DRAWLANESHAPE_H
#define DRAWLANESHAPE_H

#include "drawshape.h"
#include "baseAlgorithm/curvealgorithm.h"

class DrawLaneShape : public DrawShape
{
    Q_OBJECT
public:
    DrawLaneShape(MarkDataType dataType, bool isSegment,
                  bool isCurveFit, QObject *parent = 0);
    ~DrawLaneShape() override;

    void initDraw() override;

    void setLaneWidth(const int laneWidth);

    //polygon
    int drawMousePress(const QPoint point, bool &isDraw) override;
    int drawMouseMove(const QPoint point, bool &isDraw) override;
    int drawMouseRelease(QWidget *parent, const QPoint point, const QString sampleClass, bool &isDraw) override;
    void removeShape(bool &isDraw) override;
    bool isInShape(const QPoint &point) override;

    void drawPixmap(const QString &sampleClass, const ShapeType shapeID, QPainter &painter) override;

    void setObjectList(QList<MyObject> list) override;
    void getObjectList(QList<MyObject> &list) override;

    int getObjectSize() override;

    void setSegmentImage(const MyObject &object) override;
    MyObject getSegmentImage() override;

signals:

public slots:

private:
    QPolygon getCurrentPolygon(bool &isDraw);
    int nearPolygonPoint(const QPoint point);
    void updatePolygon(const QPoint point);

    void getCurveFitPointList(const QList<QPoint> &inputKeyPoint, QList<QPoint> &result);

    void drawMark(const QList<QPoint> &pointList, const int width,
                  QPen &pen, QPainter &painter);

    void drawMaskImage(const QList<QPoint> &pointList, const int width, const QColor &color);
    void drawMaskImage(const int width);

private:

    bool nearFirstPoint;
    bool finishDrawPolygon;

    int nearPolygonIndex;
    int polygonPointIndex;
    int removePolygonIndex;
    QPoint firstPoint;
    QPolygon currentPolygon;
    QList<MyObject> listLane;
    QList<MyObject> drawPolygons;

    bool isSegment;
    QImage *maskImage;

    bool isCurveFit;
    CurveAlgorithm curveFit;

    int laneWidth;
};


#endif // DRAWLANESHAPE_H
