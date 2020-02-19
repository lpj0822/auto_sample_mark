#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "segmentshape.h"

SegmentShape::SegmentShape()
{

}

SegmentShape::~SegmentShape()
{

}

QString SegmentShape::getShapeName(int shapeId)
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

QMap<int, QString> SegmentShape::getAllShape() const
{
    return this->shape;
}

void SegmentShape::init()
{
    shape.clear();
    shape.insert(SegmentShapeType::POLYGON_SHAPE, QObject::tr("多边形"));
}

