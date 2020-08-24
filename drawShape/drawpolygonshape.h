#ifndef DRAWPOLYGONSHAPE_H
#define DRAWPOLYGONSHAPE_H

#include "drawshape.h"
#include "drawShape/drawimagemask.h"

class DrawPolygonShape : public DrawShape
{
    Q_OBJECT
public:
    DrawPolygonShape(MarkDataType dataType, bool isSegment, QObject *parent = 0);
    ~DrawPolygonShape();

    void initDraw() override;

    //polygon
    int drawMousePress(const QPoint point, bool &isDraw) override;
    int drawMouseMove(const QPoint point, bool &isDraw) override;
    int drawMouseRelease(QWidget *parent, const QPoint point, bool &isDraw) override;
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

private:

    bool nearFirstPoint;
    bool finishDrawPolygon;

    int nearPolygonIndex;
    int polygonPointIndex;
    int removePolygonIndex;
    QPoint firstPoint;
    QPolygon currentPolygon;
    QList<MyObject> listPolygon;

    bool isSegment;
    DrawImageMask drawImageMask;
};

#endif // DRAWPOLYGONSHAPE_H
