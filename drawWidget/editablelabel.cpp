#pragma execution_character_set("utf-8")
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
    for(int loop = 0; loop < ShapeType::MAX_SHAPE_TYPE; loop++)
    {
        drawList[loop]->initDraw();
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
    if(shapeID >= 0 && shapeID < ShapeType::MAX_SHAPE_TYPE)
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
        else if(object.getShapeType() == ShapeType::LANE_SEGMENT)
        {
            laneObejcts.append(object);
        }
    }
    drawList[ShapeType::RECT_SHAPE]->setObjectList(rectObejcts);
    drawList[ShapeType::LINE_SHAPE]->setObjectList(lineObejcts);
    drawList[ShapeType::POLYGON_SHAPE]->setObjectList(polygonObejcts);
    drawList[ShapeType::LANE_SEGMENT]->setObjectList(laneObejcts);
    this->sampleClass = sampleClass;
    drawPixmap();
}

QList<MyObject> EditableLabel::getObjects()
{
    QList<MyObject> allObject;
    allObject.clear();

    for(int loop = 0; loop < ShapeType::MAX_SHAPE_TYPE; loop++)
    {
        QList<MyObject> tempObject;
        tempObject.clear();
        drawList[loop]->getObjectList(tempObject);
        allObject.append(tempObject);
    }
    return allObject;
}

QList<MyObject> EditableLabel::getSegment()
{
    QList<MyObject> allObject;
    allObject.clear();
    for(int loop = 0; loop < ShapeType::MAX_SHAPE_TYPE; loop++)
    {
        if(loop == ShapeType::LANE_SEGMENT && drawList[loop]->getObjectSize() > 0)
        {
            allObject.append(drawList[loop]->getSegmentImage());
        }
    }
    return allObject;
}

void EditableLabel::drawPixmap()
{
    QPainter painter;
    tempPixmap = mp.copy();
    painter.begin(&tempPixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);

    for(int loop = 0; loop < ShapeType::MAX_SHAPE_TYPE; loop++)
    {
        drawList[loop]->drawPixmap(this->sampleClass, this->shapeType, painter);
    }

    painter.end();
    this->update();
}

void EditableLabel::initData()
{
    this->setMouseTracking(true);
    this->setCursor(Qt::CrossCursor);

    this->sampleClass = "All";

    this->shapeType = ShapeType::RECT_SHAPE;

    this->removeRectAction = new QAction(tr("删除标注"), this);

    drawList.append(new DrawRectShape());
    drawList.append(new DrawLineShape());
    drawList.append(new DrawPolygonShape());
    drawList.append(new DrawLaneShape());
}

void EditableLabel::initConnect()
{
    connect(removeRectAction, &QAction::triggered, this, &EditableLabel::slotRemoveObject);
}
