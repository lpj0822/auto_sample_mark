#ifndef DRAWPOLYGONSHAPE_H
#define DRAWPOLYGONSHAPE_H

#include <QObject>
#include "myobject.h"
#include "baseAlgorithm/geometryalgorithm.h"

class DrawPolygonShape : public QObject
{
    Q_OBJECT
public:
    DrawPolygonShape(QObject *parent = 0);
    ~DrawPolygonShape();

    void initDraw();

    //polygon
    int drawPolygonMousePress(const QPoint point, bool &isDraw);
    int drawPolygonMouseMove(const QPoint point, bool &isDraw);
    int drawPolygonMouseRelease(QWidget *parent, const QPoint point, const QString sampleClass, bool &isDraw);

    bool polygonListContains(const QPoint point);
    void removePolygon(bool &isDraw);

    QPolygon getCurrentPolygon(bool &isDraw);
    void getPolygonList(QList<MyObject> &list);
    void setPolygonList(QList<MyObject> list);

signals:

public slots:

private:

    int nearPolygonPoint(const QPoint point);
    void updatePolygon(const QPoint point);

private:

    bool drawMousePressed;
    bool moveMousePressed;

    bool nearFirstPoint;
    bool finishDrawPolygon;

    int nearPolygonIndex;
    int polygonPointIndex;
    int removePolygonIndex;
    QPoint firstPoint;
    QPolygon currentPolygon;
    QList<MyObject> listPolygon;

    GeometryAlgorithm geometryAlgorithm;
};

#endif // DRAWPOLYGONSHAPE_H
