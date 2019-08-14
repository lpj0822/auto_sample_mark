#include "drawshape.h"

DrawShape::DrawShape(QObject *parent) : QObject(parent)
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
