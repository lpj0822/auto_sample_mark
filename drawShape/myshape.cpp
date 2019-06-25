#pragma execution_character_set("utf-8")
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

void MyShape::init()
{
    shape.clear();
    shape.insert(ShapeType::RECT_SHAPE, QObject::tr("矩形"));
    shape.insert(ShapeType::LINE_SHAPE, QObject::tr("直线"));
    shape.insert(ShapeType::POLYGON_SHAPE, QObject::tr("多边形"));
}
