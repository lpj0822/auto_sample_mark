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

    //polygon
    int drawMousePress(const QPoint point, bool &isDraw) override;
    int drawMouseMove(const QPoint point, bool &isDraw) override;
    int drawMouseRelease(QWidget *parent, const QPoint point, bool &isDraw) override;

    int drawMouseDoubleClick(QWidget *parent, const QPoint point, bool &isDraw) override;

    void removeShape(bool &isDraw) override;
    bool isInShape(const QPoint &point) override;

    void drawPixmap(const ShapeType shapeID, QPainter &painter) override;

    void setObjectList(QList<MyObject> list) override;
    void getObjectList(QList<MyObject> &list) override;

    int getObjectSize() override;

    void createImageMask(QImage &maskImage) override;

signals:

public slots:

private:
    QPolygon getCurrentPolygon(bool &isDraw);
    int nearPolygonPoint(const QPoint point);
    void updatePolygon(const QPoint point);

    void getCurveFitPointList(const QList<QPoint> &inputKeyPoint, QList<QPoint> &result);

    void drawMark(const QList<QPoint> &pointList, QPainter &painter);

    void drawMaskImage(const QList<QPoint> &pointList, const int width,
                       const QColor &color, QImage &maskImage);

private:

    bool nearFirstPoint;

    int nearPolygonIndex;
    int polygonPointIndex;
    int removePolygonIndex;
    QPoint firstPoint;
    QPolygon currentPolygon;
    QList<MyObject> listLane;

    bool isSegment;
    bool isCurveFit;
    CurveAlgorithm curveFit;
};


#endif // DRAWLANESHAPE_H
