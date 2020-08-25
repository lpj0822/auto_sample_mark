#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "editablelabel.h"
#include <QDebug>
#include <QMenu>
#include <QMessageBox>
#include "sampleMarkParam/manualparamterconfig.h"

EditableLabel::EditableLabel(QWidget *parent):
    ImageDrawLabel(parent)
{
    initData();
}

EditableLabel::~EditableLabel()
{
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

void EditableLabel::clearDraw()
{
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        drawList[drawIterator.key()]->initDraw();
    }
    drawPixmap();
}

void EditableLabel::setNewQImage(QImage &image)
{
    mp = QPixmap::fromImage(image);
    this->zoomValue = 100;
    drawPixmap();
}

void EditableLabel::setDrawShape(int shapeID)
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

void EditableLabel::resetDraw()
{
    this->zoomValue = 100;
    drawPixmap();
}

void EditableLabel::contextMenuEvent(QContextMenuEvent * event)
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

void EditableLabel::mousePressEvent(QMouseEvent *e)
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
    QLabel::mousePressEvent(e);
}

void EditableLabel::mouseMoveEvent(QMouseEvent *e)
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
    QLabel::mouseMoveEvent(e);
}

void EditableLabel::mouseReleaseEvent(QMouseEvent *e)
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
    QLabel::mouseReleaseEvent(e);
}

void EditableLabel::mouseDoubleClickEvent(QMouseEvent *event)
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

void EditableLabel::wheelEvent(QWheelEvent * event)
{
    if(event->delta() > 0)
    {
        this->zoomValue++;
        if(this->zoomValue > ManualParamterConfig::getMaxScale())
        {
            this->zoomValue = ManualParamterConfig::getMaxScale();
        }
    }
    else
    {
        this->zoomValue--;
        if(this->zoomValue < ManualParamterConfig::getMinSacle())
        {
            this->zoomValue = ManualParamterConfig::getMinSacle();
        }
    }
    drawPixmap();
    QWidget::wheelEvent(event);
}

void EditableLabel::paintEvent(QPaintEvent *e)
{
    QLabel::paintEvent(e);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::HighQualityAntialiasing);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    float scale = this->zoomValue / 100.0f;
    painter.scale(scale, scale);
    painter.translate(this->offsetToCenter());
    painter.drawPixmap(QPoint(0,0), tempPixmap);
    painter.end();
    this->resize(tempPixmap.width() * scale, tempPixmap.height() * scale);
    this->setAutoFillBackground(true);
    QLabel::paintEvent(e);
}

void EditableLabel::setDrawShapeObjects()
{
    drawList[ShapeType::RECT_SHAPE]->setObjectList(rectObejcts);
    drawList[ShapeType::LINE_SHAPE]->setObjectList(lineObejcts);
    drawList[ShapeType::POLYGON_SHAPE]->setObjectList(polygonObejcts);
    drawList[ShapeType::POLYLINE_SHAPE]->setObjectList(laneObejcts);
    drawPixmap();
}

void EditableLabel::drawPixmap()
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
    painter.end();
    this->update();
}

QPointF EditableLabel::offsetToCenter()
{
    QSize area = this->size();
    float scale = this->zoomValue / 100.0f;
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

QPoint EditableLabel::scalePoint(const QPoint point)
{
    float scale = this->zoomValue / 100.0f;
    QPoint resultPoint(static_cast<int>(point.x() / scale),
                       static_cast<int>(point.y() / scale));
    return resultPoint;
}

void EditableLabel::initData()
{
    this->setMouseTracking(true);
    this->setCursor(Qt::CrossCursor);

    this->shapeType = ShapeType::RECT_SHAPE;

    this->zoomValue = 100;

    drawList.clear();
    drawList.insert(ShapeType::RECT_SHAPE, new DrawRectShape(MarkDataType::IMAGE));
    drawList.insert(ShapeType::LINE_SHAPE, new DrawLineShape(MarkDataType::IMAGE));
    drawList.insert(ShapeType::POLYGON_SHAPE, new DrawPolygonShape(MarkDataType::IMAGE, false));
    drawList.insert(ShapeType::POLYLINE_SHAPE, new DrawLaneShape(MarkDataType::IMAGE, false));
    QMap<ShapeType, DrawShape*>::const_iterator drawIterator;
    for(drawIterator = drawList.constBegin(); drawIterator != drawList.constEnd(); ++drawIterator)
    {
        drawList[drawIterator.key()]->setVisibleSampleClass(this->sampleClass);
    }
}
