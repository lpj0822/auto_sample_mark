#ifndef DRAWLANESHAPE_H
#define DRAWLANESHAPE_H

#include "drawshape.h"
#include "baseAlgorithm/curvealgorithm.h"

class DrawLaneShape : public DrawShape
{
    Q_OBJECT
public:
    DrawLaneShape(QObject *parent = 0);
    ~DrawLaneShape();

    void initDraw() override;

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

    void drawMark(const QList<QPoint> &pointList, QPen &pen, QPainter &painter);

    void drawMaskImage(const QList<QPoint> &pointList, const QColor &color);

private:

    bool nearFirstPoint;
    bool finishDrawPolygon;

    int nearPolygonIndex;
    int polygonPointIndex;
    int removePolygonIndex;
    QPoint firstPoint;
    QPolygon currentPolygon;
    QList<MyObject> listLane;
    QImage *maskImage;
    CurveAlgorithm curveFit;
};

#endif // DRAWLANESHAPE_H
