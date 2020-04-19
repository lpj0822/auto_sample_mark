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

    //菜单出现的位置为当前鼠标的位置
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
            this->setCursor(Qt::CrossCursor);
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
        this->setCursor(Qt::CrossCursor);
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
        if(object.getShapeType() == ShapeType::POLYGON_SHAPE)
        {
            polygonObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::LANE_SEGMENT)
        {
            laneObejcts.append(object);
        }
    }
    drawList[ShapeType::POLYGON_SHAPE]->setObjectList(polygonObejcts);
    drawList[ShapeType::LANE_SEGMENT]->setObjectList(laneObejcts);
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
    if(shapeType == ShapeType::LANE_SEGMENT)
    {
        if(drawList[ShapeType::LANE_SEGMENT]->getObjectSize() > 0)
        {
            result = drawList[ShapeType::LANE_SEGMENT]->getSegmentImage();
        }
    }
    else
    {
        if(maskImage != NULL)
            result.setSegmentImage(*maskImage);
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
    if(shapeType != ShapeType::LANE_SEGMENT)
        drawSegmentMask(painter);
    painter.end();
    this->update();
}

void SegmentLabel::drawSegmentMask(QPainter &painter)
{
    if(shapeType == ShapeType::POLYGON_SHAPE)
    {
        const int height = painter.device()->height();
        const int width = painter.device()->width();
        QList<MyObject> allObject = getObjects();
        if(allObject.count() > 0)
        {
            if(maskImage != NULL)
            {
                delete maskImage;
                maskImage = NULL;
            }
            QImage result = segmentPorcess.generateMaskFromPolygon(allObject, width, height);
            maskImage = new QImage(result);
        }
    }
    if(maskImage != NULL)
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
    this->setMouseTracking(true);
    this->setCursor(Qt::CrossCursor);

    this->scale = 1.0f;
    this->maskImage = NULL;
    this->sampleClass = "All";

     this->removeRectAction = new QAction(tr("删除标注"), this);

    this->shapeType = ShapeType::UNSHAPE;
    drawList.clear();
    drawList.insert(ShapeType::POLYGON_SHAPE, new DrawPolygonShape(MarkDataType::SEGMENT));
    drawList.insert(ShapeType::LANE_SEGMENT, new DrawLaneShape(MarkDataType::SEGMENT));
}

void SegmentLabel::initConnect()
{
    connect(removeRectAction, &QAction::triggered, this, &SegmentLabel::slotRemoveObject);
}
