#ifndef DRAWPOLYGONSHAPE_H
#define DRAWPOLYGONSHAPE_H

#include "drawshape.h"

class DrawPolygonShape : public DrawShape
{
    Q_OBJECT
public:
    DrawPolygonShape(QObject *parent = 0);
    ~DrawPolygonShape();

    void initDraw();

    //polygon
    int drawMousePress(const QPoint point, bool &isDraw);
    int drawMouseMove(const QPoint point, bool &isDraw);
    int drawMouseRelease(QWidget *parent, const QPoint point, const QString sampleClass, bool &isDraw);
    void removeShape(bool &isDraw);
    bool isInShape(const QPoint &point);

    void drawPixmap(const QString &sampleClass, const ShapeType shapeID, QPainter &painter);

    void setObjectList(QList<MyObject> list);
    void getObjectList(QList<MyObject> &list);

    QPolygon getCurrentPolygon(bool &isDraw);

signals:

public slots:

private:

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
};

#endif // DRAWPOLYGONSHAPE_H
