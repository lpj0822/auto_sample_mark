#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "segmentlabel.h"
#include <QMenu>
#include <QMessageBox>
#include <QDebug>
#include "sampleMarkParam/manualparamterconfig.h"

SegmentLabel::SegmentLabel(QWidget *parent):
    QLabel(parent)
{
    initData();
    initConnect();
}

SegmentLabel::~SegmentLabel()
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        if(drawList[drawIterator.key()] != NULL)
        {
            delete drawList[drawIterator.key()];
            drawList[drawIterator.key()] = NULL;
        }
    }
}

void SegmentLabel::slotRemoveObject()
{
    bool isDraw = false;
    drawList[this->shapeType]->removeShape(isDraw);
    if(isDraw)
    {
        drawPixmap();
    }
}

void SegmentLabel::contextMenuEvent(QContextMenuEvent * event)
{
    QMenu* popMenu = new QMenu(this);
    bool isFind = drawList[this->shapeType]->isInShape(this->mapFromGlobal(QCursor::pos()));
    if(isFind)
    {
        popMenu->addAction(removeRectAction);
    }

    popMenu->exec(QCursor::pos());
    QLabel::contextMenuEvent(event);
}

void SegmentLabel::mousePressEvent(QMouseEvent *e)
{
    if(!this->rect().contains(e->pos()))
    {
        return;
    }

    if(e->button() == Qt::LeftButton)
    {
        bool isDraw = false;
        int mouseChange = drawList[this->shapeType]->drawMousePress(e->pos(), isDraw);
        if(mouseChange == 1)
        {
            this->setCursor(myDrawCursor);
        }
        else if(mouseChange == 2)
        {
            this->setCursor(Qt::SizeAllCursor);
        }
        if(isDraw)
        {
            drawPixmap();
        }
    }
}

void SegmentLabel::mouseMoveEvent(QMouseEvent *e)
{
    if(!this->rect().contains(e->pos()))
    {
        return;
    }
    bool isDraw = false;
    int mouseChange = drawList[this->shapeType]->drawMouseMove(e->pos(), isDraw);
    if(mouseChange == 1)
    {
        this->setCursor(myDrawCursor);
    }
    else if(mouseChange == 2)
    {
        this->setCursor(Qt::SizeAllCursor);
    }
    if(isDraw)
    {
        drawPixmap();
    }
}

void SegmentLabel::mouseReleaseEvent(QMouseEvent *e)
{
    if(!this->rect().contains(e->pos()))
    {
        return;
    }

    if(e->button() == Qt::LeftButton)
    {
        bool isDraw = false;
        drawList[this->shapeType]->drawMouseRelease(this, e->pos(), this->sampleClass, isDraw);
        if(isDraw)
        {
            drawPixmap();
        }
    }
}

void SegmentLabel::wheelEvent(QWheelEvent * event)
{
//    int value = event->delta();
//    float scaleRatio = value / (8 * 360.0f);
//    scale += scaleRatio;
//    if(scale > 2.0f)
//    {
//        scale = 2.0f;
//    }
//    else if(scale < 0.5f)
//    {
//        scale = 0.5f;
//    }
//    drawPixmap();
    QLabel::wheelEvent(event);
}

void SegmentLabel::paintEvent(QPaintEvent *e)
{
    QLabel::paintEvent(e);
    QPainter painter(this);
    int width = static_cast<int>(tempPixmap.width() * scale);
    int height = static_cast<int>(tempPixmap.height() * scale);
    QPixmap temp = tempPixmap.scaled(width, height, Qt::KeepAspectRatio);
    painter.drawPixmap(QPoint(0,0), temp);
    painter.end();
    this->resize(temp.width(), temp.height());
}

void SegmentLabel::clearObjects()
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
    this->scale = 1.0f;
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        drawList[drawIterator.key()]->initDraw();
    }
    drawPixmap();
}

void SegmentLabel::setNewQImage(QImage &image)
{
    mp = QPixmap::fromImage(image);
    drawPixmap();
}

void SegmentLabel::setDrawShape(int shapeID)
{
    if(shapeID >= 0 && shapeID < ShapeType::MAX_IMAGE_SHAPE_TYPE)
    {
        this->shapeType = static_cast<ShapeType>(shapeID);
        drawPixmap();
    }
    else
    {
        QMessageBox::information(this, tr("标注形状"), tr("选择的标注形状有误！"));
    }
}

void SegmentLabel::setOjects(const MyObject &mask, const QList<MyObject> &obejcts, const QString &sampleClass)
{
    QList<MyObject> polygonObejcts;
    QList<MyObject> laneObejcts;
    polygonObejcts.clear();
    laneObejcts.clear();
    for(int loop = 0; loop < obejcts.count(); loop++)
    {
        const MyObject object = obejcts[loop];
        if(object.getShapeType() == ShapeType::SEGMENT_POLYGON_SHAPE)
        {
            polygonObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::LANE_SHAPE)
        {
            laneObejcts.append(object);
        }
    }
    drawList[ShapeType::SEGMENT_POLYGON_SHAPE]->setObjectList(polygonObejcts);
    drawList[ShapeType::LANE_SHAPE]->setObjectList(laneObejcts);
    setMaskOject(mask);
    this->sampleClass = sampleClass;
    drawPixmap();
}

QList<MyObject> SegmentLabel::getObjects()
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

MyObject SegmentLabel::getSegmentMask()
{
    MyObject result;
    if(shapeType == ShapeType::LANE_SHAPE)
    {
        if(drawList[ShapeType::LANE_SHAPE]->getObjectSize() > 0)
        {
            result = drawList[ShapeType::LANE_SHAPE]->getSegmentImage();
        }
    }
    else if(shapeType == ShapeType::SEGMENT_POLYGON_SHAPE)
    {
        if(drawList[ShapeType::SEGMENT_POLYGON_SHAPE]->getObjectSize() > 0)
        {
            result = drawList[ShapeType::SEGMENT_POLYGON_SHAPE]->getSegmentImage();
        }
    }
    return result;
}

void SegmentLabel::drawPixmap()
{
    QPainter painter;
    tempPixmap = mp.copy();
    painter.begin(&tempPixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        drawList[drawIterator.key()]->drawPixmap(this->sampleClass, this->shapeType, painter);
    }
    drawSegmentMask(painter);
    painter.end();
    this->update();
}

void SegmentLabel::drawSegmentMask(QPainter &painter)
{
    QList<MyObject> result = getObjects();
    if(maskImage != NULL && result.count() == 0)
    {
        painter.save();
        painter.setOpacity(0.5);
        painter.drawImage(QPoint(0, 0), *maskImage);
        painter.restore();
    }
}

void SegmentLabel::setMaskOject(const MyObject &mask)
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
    if(!mask.getSegmentImage().isNull())
        maskImage = new QImage(mask.getSegmentImage());
}

void SegmentLabel::initData()
{
    myDrawCursor = QCursor(QPixmap(tr(":/images/images/cross.png")));

    this->setMouseTracking(true);
    this->setCursor(myDrawCursor);

    this->scale = 1.0f;
    this->maskImage = NULL;
    this->sampleClass = "All";

    this->removeRectAction = new QAction(tr("删除标注"), this);

    this->shapeType = ShapeType::SEGMENT_POLYGON_SHAPE;
    drawList.clear();
    drawList.insert(ShapeType::LANE_SHAPE,
                    new DrawLaneShape(MarkDataType::SEGMENT, true, true));
    drawList.insert(ShapeType::SEGMENT_POLYGON_SHAPE,
                    new DrawPolygonShape(MarkDataType::SEGMENT, true));
}

void SegmentLabel::initConnect()
{
    connect(removeRectAction, &QAction::triggered, this, &SegmentLabel::slotRemoveObject);
}
