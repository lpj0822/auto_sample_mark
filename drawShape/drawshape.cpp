#include "drawshape.h"

DrawShape::DrawShape(MarkDataType dataType, QObject *parent) :
    QObject(parent), markDataType(dataType)
{

}

int DrawShape::getObjectSize()
{
    return 0;
}

void DrawShape::setSegmentImage(const MyObject &object)
{

}

MyObject DrawShape::getSegmentImage()
{
    return MyObject();
}
