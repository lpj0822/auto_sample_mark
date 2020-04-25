#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "editablelabel.h"
#include <QDebug>
#include <QMenu>
#include <QMessageBox>
#include "sampleMarkParam/manualparamterconfig.h"

EditableLabel::EditableLabel(QWidget *parent):
    QLabel(parent)
{
    initData();
    initConnect();
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

void EditableLabel::slotRemoveObject()
{
    bool isDraw = false;
    drawList[this->shapeType]->removeShape(isDraw);
    if(isDraw)
    {
        drawPixmap();
    }
}

void EditableLabel::contextMenuEvent(QContextMenuEvent * event)
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

void EditableLabel::mousePressEvent(QMouseEvent *e)
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

void EditableLabel::mouseMoveEvent(QMouseEvent *e)
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

void EditableLabel::mouseReleaseEvent(QMouseEvent *e)
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

void EditableLabel::wheelEvent(QWheelEvent * event)
{
//    if(event->delta() > 0)
//    {
//        scale++;//放大
//        if(scale > MAX_SCALE)
//        {
//            scale = MAX_SCALE;
//        }
//    }
//    else
//    {
//        scale--;//缩小
//        if(scale < MIN_SCALE)
//        {
//            scale = MIN_SCALE;
//        }
//    }
//    drawPixmap();
    QWidget::wheelEvent(event);
}

void EditableLabel::paintEvent(QPaintEvent *e)
{
    QLabel::paintEvent(e);
    QPainter painter(this);
    //painter.save(); //保存坐标系状态
    //painter.scale(scale / 100.0, scale / 100.0);
    painter.drawPixmap(QPoint(0,0), tempPixmap);
    //painter.restore(); //恢复以前的坐标系状态
    painter.end();
    this->resize(tempPixmap.width(), tempPixmap.height());
}

void EditableLabel::clearObjects()
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

void EditableLabel::setOjects(QList<MyObject> obejcts, QString sampleClass)
{
    QList<MyObject> rectObejcts;
    QList<MyObject> lineObejcts;
    QList<MyObject> polygonObejcts;
    QList<MyObject> laneObejcts;
    rectObejcts.clear();
    lineObejcts.clear();
    polygonObejcts.clear();
    laneObejcts.clear();
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
        else if(object.getShapeType() == ShapeType::LANE_SHAPE)
        {
            laneObejcts.append(object);
        }
    }
    drawList[ShapeType::RECT_SHAPE]->setObjectList(rectObejcts);
    drawList[ShapeType::LINE_SHAPE]->setObjectList(lineObejcts);
    drawList[ShapeType::POLYGON_SHAPE]->setObjectList(polygonObejcts);
    drawList[ShapeType::LANE_SHAPE]->setObjectList(laneObejcts);
    this->sampleClass = sampleClass;
    drawPixmap();
}

QList<MyObject> EditableLabel::getObjects()
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

void EditableLabel::drawPixmap()
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
    painter.end();
    this->update();
}

void EditableLabel::initData()
{
    this->setMouseTracking(true);
    this->setCursor(Qt::CrossCursor);

    this->sampleClass = "All";

    this->removeRectAction = new QAction(tr("删除标注"), this);

    this->shapeType = ShapeType::RECT_SHAPE;

    drawList.clear();
    drawList.insert(ShapeType::RECT_SHAPE, new DrawRectShape(MarkDataType::IMAGE));
    drawList.insert(ShapeType::LINE_SHAPE, new DrawLineShape(MarkDataType::IMAGE));
    drawList.insert(ShapeType::POLYGON_SHAPE, new DrawPolygonShape(MarkDataType::IMAGE, false));
    drawList.insert(ShapeType::LANE_SHAPE, new DrawLaneShape(MarkDataType::IMAGE, false, false));
}

void EditableLabel::initConnect()
{
    connect(removeRectAction, &QAction::triggered, this, &EditableLabel::slotRemoveObject);
}
