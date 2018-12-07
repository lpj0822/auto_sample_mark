#include "geometryalgorithm.h"
#include <algorithm>

GeometryAlgorithm::GeometryAlgorithm()
{

}

GeometryAlgorithm::~GeometryAlgorithm()
{

}

float GeometryAlgorithm::rectOverlap(const QRect& box1, const QRect& box2)
{
    float intersection = overlappingArea(box1, box2);
    float area1 = static_cast<float>(box1.width() * box1.height());
    float area2 = static_cast<float>(box2.width() * box2.height());
    if(area2 <= 0 || area2 <= 0)
    {
        return 0.0f;
    }
    else
    {
        return intersection / (area1 + area2 - intersection);
    }
}

bool GeometryAlgorithm::onSegment(const QPoint Pi , const QPoint Pj , const QPoint Q)
{
    int minX  = std::min(Pi.x(), Pj.x());
    int maxX = std::max(Pi.x(), Pj.x());
    int minY = std::min(Pi.y(), Pj.y());
    int maxY = std::max(Pi.y(), Pj.y());
    if((Q.x() - Pi.x()) * (Pj.y() - Pi.y()) == (Pj.x() - Pi.x()) * (Q.y() - Pi.y())  //叉乘
       //保证Q点坐标在pi,pj之间
       && minX <= Q.x() && Q.x() <= maxX
       && minY <= Q.y() && Q.y() <= maxY)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool GeometryAlgorithm::pointInPolygon(const QPoint point, const QPolygon& polygon)
{
    bool isIn = false;
    int count = polygon.count();
    int i = 0, j = count - 1;
    if (count <= 2)
        return false;

    for(i = 0; i < count; i++)
    {
        if((polygon[i].y() < point.y() && polygon[j].y() >= point.y() ||
            polygon[j].y() < point.y() && polygon[i].y () >= point.y())
                &&(polygon[i].x() <= point.x() || polygon[j].x() <= point.x()))
        {
          if(polygon[i].x() + (point.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) * (polygon[j].x() - polygon[i].x()) < point.x())
          {
              isIn = !isIn;
          }
        }
        j = i;
    }
    return isIn;
}

float GeometryAlgorithm::pointToLineDistance(const QPoint Pi , const QPoint Pj , const QPoint Q)
{
    float distance = 0;
    float a = 0, b = 0, c = 0;
    float p = 0;
    float s = 0;
    a = lineDistance(Pi, Pj);
    b = lineDistance(Pi, Q);
    c = lineDistance(Pj, Q);
    if (c <= 0.000001f || b <= 0.000001f)
    {
       distance = 0;
       return distance;
    }
    if (a <= 0.000001f)
    {
       distance = b;
       return distance;
    }
    if (c * c >= a * a + b * b)
    {
       distance = b;
       return distance;
    }
    if (b * b >= a * a + c * c)
    {
       distance = c;
       return distance;
    }
    p = (a + b + c) / 2.0f;// 半周长
    s = std::sqrt(p * (p - a) * (p - b) * (p - c));// 海伦公式求面积
    distance = 2 * s / a;// 返回点到线的距离（利用三角形面积公式求高）
    return distance;
}

float GeometryAlgorithm::lineDistance(const QPoint pointI , const QPoint pointJ)
{
    QPoint segment = pointI - pointJ;
    float distance = (float)(std::sqrt(segment.x() * segment.x() + segment.y() * segment.y()));
    return distance;
}

//得到多边形的中心
QPoint GeometryAlgorithm::getPolygonCenter(const QPolygon& polygon)
{
    QPoint center;
    int count = polygon.count();
    float x = 0;
    float y = 0;
    for (int i = 0 ; i < count ; i++ )
    {
        x += polygon[i].x();
        y += polygon[i].y();
    }
    x /= count;
    y /= count;
    center.setX((int)x);
    center.setY((int)y);
    return center;
}

//计算多边形的面积
float GeometryAlgorithm::polygonArea(const QPolygon& polygon)
{
    float tempArea = 0.0f;
    int count = polygon.count();
    for (int i = 0 ; i < count ; i++ )
    {
        int j = (i + 1) % count;
        tempArea += polygon[i].x() * polygon[j].y();
        tempArea -= polygon[i].y() * polygon[j].x();
    }
    return 0.5f * std::fabs(tempArea);
}

float GeometryAlgorithm::overlappingArea(const QRect& box1, const QRect& box2)
{
    int width = std::min(box1.bottomRight().x(), box2.bottomRight().x()) - \
            std::max(box1.topLeft().x(), box2.topLeft().x());
    int height = std::min(box1.bottomRight().y(), box2.bottomRight().y()) - \
            std::max(box1.topLeft().y(), box2.topLeft().y());
    if(width < 0 || height < 0)
    {
        return 0.0f;
    }
    else
    {
        return (float)width * height;
    }
}
