#include "drawshape.h"

DrawShape::DrawShape(MarkDataType dataType, QObject *parent) :
    QObject(parent), markDataType(dataType)
{

}

int DrawShape::drawMouseDoubleClick(QWidget *parent, const QPoint point, bool &isDraw)
{
    isDraw = false;
    return 0;
}

void DrawShape::cancelDrawShape(bool &isDraw)
{
    isDraw = false;
}

int DrawShape::getObjectSize()
{
    return 0;
}

void DrawShape::createImageMask(QImage &maskImage)
{

}

void DrawShape::setVisibleSampleClass(const QString &sampleClass)
{
    this->visibleSampleClass = sampleClass;
}
