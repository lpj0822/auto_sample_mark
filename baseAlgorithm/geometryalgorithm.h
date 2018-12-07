#ifndef GEOMETRYALGORITHM_H
#define GEOMETRYALGORITHM_H

#include <QPolygon>
#include <QPoint>
#include <QRect>

class GeometryAlgorithm
{
public:
    GeometryAlgorithm();
    ~GeometryAlgorithm();

    float rectOverlap(const QRect& box1, const QRect& box2);//计算两个矩形IOU

    bool onSegment(const QPoint Pi , const QPoint Pj , const QPoint Q);

    bool pointInPolygon(const QPoint point, const QPolygon& polygon);

    float pointToLineDistance(const QPoint Pi , const QPoint Pj , const QPoint Q);

    float lineDistance(const QPoint pointI , const QPoint pointJ);

    //得到多边形的中心
    QPoint getPolygonCenter(const QPolygon& polygon);

    //计算多边形的面积
    float polygonArea(const QPolygon& polygon);

    float overlappingArea(const QRect& box1, const QRect& box2);
};

#endif // GEOMETRYALGORITHM_H
