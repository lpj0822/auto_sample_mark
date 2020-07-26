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
    QPoint point = this->mapFromGlobal(QCursor::pos());
    bool isFind = drawList[this->shapeType]->isInShape(scalePoint(point));
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
        QPoint point = e->pos();
        int mouseChange = drawList[this->shapeType]->drawMousePress(scalePoint(point), isDraw);
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
    QPoint point = e->pos();
    int mouseChange = drawList[this->shapeType]->drawMouseMove(scalePoint(point), isDraw);
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
        QPoint point = e->pos();
        drawList[this->shapeType]->setVisibleSampleClass(this->sampleClass);
        drawList[this->shapeType]->drawMouseRelease(this, scalePoint(point), isDraw);
        if(isDraw)
        {
            drawPixmap();
        }
    }
}

void SegmentLabel::mouseDoubleClickEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        bool isDraw = false;
        QPoint point = event->pos();
        drawList[this->shapeType]->setVisibleSampleClass(this->sampleClass);
        drawList[this->shapeType]->drawMouseDoubleClick(this, scalePoint(point), isDraw);
        if(isDraw)
        {
            drawPixmap();
        }
    }
    //QLabel::mouseDoubleClickEvent(event);
}

void SegmentLabel::wheelEvent(QWheelEvent * event)
{
    int value = event->delta();
    float scaleRatio = value / (8 * 360.0f);
    scale += scaleRatio;
    if(scale > 2.0f)
    {
        scale = 2.0f;
    }
    else if(scale < 0.5f)
    {
        scale = 0.5f;
    }
    drawPixmap();
    QLabel::wheelEvent(event);
}

void SegmentLabel::paintEvent(QPaintEvent *e)
{
    QLabel::paintEvent(e);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::HighQualityAntialiasing);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.scale(this->scale, this->scale);
    painter.translate(this->offsetToCenter());
    painter.drawPixmap(QPoint(0,0), tempPixmap);
    painter.end();
    this->resize(tempPixmap.width() * this->scale, tempPixmap.height() * this->scale);
    this->setAutoFillBackground(true);
    QLabel::paintEvent(e);
}

void SegmentLabel::clearObjects()
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
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
    this->scale = 1.0f;
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
    if(maskImage != NULL)
        result.setSegmentImage(*maskImage);
    return result;
}

void SegmentLabel::resetDraw()
{
    this->scale = 1.0f;
    drawPixmap();
}

void SegmentLabel::drawPixmap()
{
    QPainter painter;
    tempPixmap = mp.copy();
    painter.begin(&tempPixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::HighQualityAntialiasing);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        drawList[drawIterator.key()]->setVisibleSampleClass(this->sampleClass);
        drawList[drawIterator.key()]->drawPixmap(this->shapeType, painter);
    }
    drawSegmentMask(painter);
    painter.end();
    this->update();
}

void SegmentLabel::drawSegmentMask(QPainter &painter)
{
    const int height = painter.device()->height();
    const int width = painter.device()->width();
    QList<MyObject> result = getObjects();
    if(!this->isEnabled() && maskImage != NULL && result.count() == 0)
    {
        painter.save();
        painter.setOpacity(0.35);
        painter.drawImage(QPoint(0, 0), *maskImage);
        painter.restore();
    }
    else
    {
        if(maskImage != NULL)
        {
            delete maskImage;
            maskImage = NULL;
        }
        maskImage = new QImage(width, height, QImage::Format_ARGB32);
        maskImage->fill(QColor(255, 255, 255));
        QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
        for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
        {
            drawList[drawIterator.key()]->setVisibleSampleClass(this->sampleClass);
            drawList[drawIterator.key()]->createImageMask(*maskImage);
        }
        if(maskImage != NULL)
        {
            painter.save();
            painter.setOpacity(0.35);
            painter.drawImage(QPoint(0, 0), *maskImage);
            painter.restore();
        }
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

QPointF SegmentLabel::offsetToCenter()
{
    QSize area = this->size();
    float w = this->mp.width() * scale;
    float h = this->mp.height() * scale;
    float aw = area.width();
    float ah = area.height();
    float x = 0;
    float y = 0;
    if(aw > w)
    {
        x = (aw - w) / (2 * scale);
    }
    if(ah > h)
    {
        y = (ah - h) / (2 * scale);
    }
    return QPointF(x, y);
}

QPoint SegmentLabel::scalePoint(const QPoint point)
{
    QPoint resultPoint(static_cast<int>(point.x() / scale),
                       static_cast<int>(point.y() / scale));
    return resultPoint;
}

void SegmentLabel::initData()
{
    myDrawCursor = QCursor(QPixmap(tr(":/images/images/cross.png")));

    this->setMouseTracking(true);
    this->setCursor(myDrawCursor);

    this->scale = 1.0f;
    this->sampleClass = "All";

    this->maskImage = NULL;

    this->removeRectAction = new QAction(tr("删除标注"), this);

    this->shapeType = ShapeType::SEGMENT_POLYGON_SHAPE;
    drawList.clear();
    drawList.insert(ShapeType::SEGMENT_POLYGON_SHAPE,
                    new DrawPolygonShape(MarkDataType::SEGMENT, true));
    drawList.insert(ShapeType::LANE_SHAPE,
                    new DrawLaneShape(MarkDataType::SEGMENT, true, true));
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        drawList[drawIterator.key()]->setVisibleSampleClass(this->sampleClass);
    }
}

void SegmentLabel::initConnect()
{
    connect(removeRectAction, &QAction::triggered, this, &SegmentLabel::slotRemoveObject);
}
