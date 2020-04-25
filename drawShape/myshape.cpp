#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "myshape.h"

MyShape::MyShape()
{
    init();
}

MyShape::~MyShape()
{

}

QString MyShape::getShapeName(int shapeId)
{
    if(this->shape.contains(shapeId))
    {
        return this->shape[shapeId];
    }
    else
    {
        return "";
    }
}

QMap<int, QString> MyShape::getAllShape() const
{
    return this->shape;
}

QMap<int, QString> MyShape::getImageShape() const
{
    QMap<int, QString> result;
    result.clear();
    result.insert(ShapeType::RECT_SHAPE, this->shape[ShapeType::RECT_SHAPE]);
    result.insert(ShapeType::LINE_SHAPE, this->shape[ShapeType::LINE_SHAPE]);
    result.insert(ShapeType::POLYGON_SHAPE, this->shape[ShapeType::POLYGON_SHAPE]);
    result.insert(ShapeType::LANE_SHAPE, this->shape[ShapeType::LANE_SHAPE]);
    return result;
}

QMap<int, QString> MyShape::getSegmentShape() const
{
    QMap<int, QString> result;
    result.clear();
    result.insert(ShapeType::POLYGON_SHAPE, QObject::tr("多边形分割"));
    result.insert(ShapeType::LANE_SHAPE, QObject::tr("折线分割"));
    return result;
}

void MyShape::init()
{
    shape.clear();
    shape.insert(ShapeType::RECT_SHAPE, QObject::tr("矩形"));
    shape.insert(ShapeType::LINE_SHAPE, QObject::tr("直线"));
    shape.insert(ShapeType::POLYGON_SHAPE, QObject::tr("多边形"));
    shape.insert(ShapeType::LANE_SHAPE, QObject::tr("折线"));
}
