#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "imagedrawlabel.h"

ImageDrawLabel::ImageDrawLabel(QWidget *parent):
    QLabel(parent)
{
    init();
}

ImageDrawLabel::~ImageDrawLabel()
{

}

void ImageDrawLabel::setOjects(QList<MyObject> obejcts, QString sampleClass)
{
    rectObejcts.clear();
    lineObejcts.clear();
    polygonObejcts.clear();
    laneObejcts.clear();

    polygonSegObejcts.clear();
    laneSegObejcts.clear();
    for(int loop = 0; loop < obejcts.count(); loop++)
    {
        const MyObject object = obejcts[loop];
        if(object.getShapeType() == ShapeType::RECT_SHAPE)
        {
            rectObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::LINE_SHAPE)
        {
            lineObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::POLYGON_SHAPE)
        {
            polygonObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::POLYLINE_SHAPE)
        {
            laneObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::POLYGON_SEGMENT_SHAPE)
        {
            polygonSegObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::LANE_SEGMENT_SHAPE)
        {
            laneSegObejcts.append(object);
        }
    }
    this->sampleClass = sampleClass;
    setDrawShapeObjects();
}

void ImageDrawLabel::setMaskOject(const MyObject &mask)
{

}

QList<MyObject> ImageDrawLabel::getObjects()
{
    QList<MyObject> allObject;
    allObject.clear();
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        QList<MyObject> tempObject;
        tempObject.clear();
        drawList[drawIterator.key()]->getObjectList(tempObject);
        allObject.append(tempObject);
    }
    return allObject;
}

void ImageDrawLabel::undoDrawShape()
{
    bool isDraw = false;
    if(drawList.count() > 0)
    {
        drawList[this->shapeType]->cancelDrawShape(isDraw);
    }
    if(isDraw)
    {
        drawPixmap();
    }
}

void ImageDrawLabel::slotRemoveObject()
{
    bool isDraw = false;
    if(drawList.count() > 0)
    {
        drawList[this->shapeType]->removeShape(isDraw);
    }
    if(isDraw)
    {
        drawPixmap();
    }
}

void ImageDrawLabel::init()
{
    this->removeRectAction = new QAction(tr("删除标注"), this);

    this->shapeType = ShapeType::UNSHAPE;
    drawList.clear();

    this->sampleClass = "All";

    connect(removeRectAction, &QAction::triggered, this, &ImageDrawLabel::slotRemoveObject);
}

