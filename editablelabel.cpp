#pragma execution_character_set("utf-8")
#include "editablelabel.h"
#include <QDebug>
#include <QMenu>
#include <QMessageBox>
#include "manualparamterconfig.h"

EditableLabel::EditableLabel(QWidget *parent):
    QLabel(parent)
{
    initData();
    initConnect();
}

void EditableLabel::slotRemoveObject()
{
    switch(this->shapeType)
    {
     case ShapeType::RECT:
        {
            bool isDraw = false;
            drawRect.removeRect(isDraw);
            if(isDraw)
            {
                drawPixmap();
            }
        }
        break;
     case ShapeType::LINE:
        {
            bool isDraw = false;
            drawLine.removeLine(isDraw);
            if(isDraw)
            {
                drawPixmap();
            }
        }
        break;
    case ShapeType::POLYGON:
       {
           bool isDraw = false;
           drawPolygon.removePolygon(isDraw);
           if(isDraw)
           {
               drawPixmap();
           }
       }
       break;
    default:
        break;
    }
}

void EditableLabel::contextMenuEvent(QContextMenuEvent * event)
{
    QMenu* popMenu = new QMenu(this);
    bool isFind = false;
    switch(this->shapeType)
    {
     case ShapeType::RECT:
        {
            isFind = drawRect.rectListContains(this->mapFromGlobal(QCursor::pos()));
        }
        break;
     case ShapeType::LINE:
        {
            isFind = drawLine.lineListContains(this->mapFromGlobal(QCursor::pos()));
        }
        break;
    case ShapeType::POLYGON:
       {
           isFind = drawPolygon.polygonListContains(this->mapFromGlobal(QCursor::pos()));
       }
       break;
    default:
        break;
    }
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
        switch(this->shapeType)
        {
         case ShapeType::RECT:
            {
                bool isDraw = false;
                int mouseChange = drawRect.drawRectMousePress(e->pos(), isDraw);
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
            break;
         case ShapeType::LINE:
            {
                bool isDraw = false;
                int mouseChange = drawLine.drawLineMousePress(e->pos(), isDraw);
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
            break;
        case ShapeType::POLYGON:
            {
                bool isDraw = false;
                int mouseChange = drawPolygon.drawPolygonMousePress(e->pos(), isDraw);
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
            break;
        }
    }
}

void EditableLabel::mouseMoveEvent(QMouseEvent *e)
{
    if(!this->rect().contains(e->pos()))
    {
        return;
    }

    switch(this->shapeType)
    {
     case ShapeType::RECT:
        {
            bool isDraw = false;
            int mouseChange = drawRect.drawRectMouseMove(e->pos(), isDraw);
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
        break;
     case ShapeType::LINE:
        {
            bool isDraw = false;
            int mouseChange = drawLine.drawLineMouseMove(e->pos(), isDraw);
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
        break;
     case ShapeType::POLYGON:
        {
            bool isDraw = false;
            int mouseChange = drawPolygon.drawPolygonMouseMove(e->pos(), isDraw);
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
       break;
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
        switch(this->shapeType)
        {
        case ShapeType::RECT:
            {
                bool isDraw = false;
                drawRect.drawRectMouseRelease(this, e->pos(), this->sampleClass, isDraw);
                if(isDraw)
                {
                    drawPixmap();
                }
            }
            break;
        case ShapeType::LINE:
            {
                bool isDraw = false;
                drawLine.drawLineMouseRelease(this, e->pos(), this->sampleClass, isDraw);
                if(isDraw)
                {
                    drawPixmap();
                }
            }
            break;
        case ShapeType::POLYGON:
            {
                bool isDraw = false;
                drawPolygon.drawPolygonMouseRelease(this, e->pos(), this->sampleClass, isDraw);
                if(isDraw)
                {
                    drawPixmap();
                }
            }
            break;
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
    drawRect.initDraw();
    drawLine.initDraw();
    drawPolygon.initDraw();
    drawPixmap();
}

void EditableLabel::setNewQImage(QImage &image)
{
    mp = QPixmap::fromImage(image);
    drawPixmap();
}

void EditableLabel::setDrawShape(int shapeID)
{
    switch(shapeID)
    {
     case ShapeType::RECT:
        this->shapeType = ShapeType::RECT;
        break;
     case ShapeType::LINE:
        this->shapeType = ShapeType::LINE;
        break;
     case ShapeType::POLYGON:
        this->shapeType = ShapeType::POLYGON;
        break;
     default:
        this->shapeType = ShapeType::UNSHAPE;
        QMessageBox::information(this, tr("标注形状"), tr("选择的标注形状有误！"));
    }
}

void EditableLabel::setOjects(QList<MyObject> obejcts, QString sampleClass)
{
    QList<MyObject> rectObejcts;
    QList<MyObject> lineObejcts;
    QList<MyObject> polygonObejcts;
    rectObejcts.clear();
    lineObejcts.clear();
    polygonObejcts.clear();
    for(int loop = 0; loop < obejcts.count(); loop++)
    {
        const MyObject object = obejcts[loop];
        if(object.getShapeType() == ShapeType::RECT)
        {
            rectObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::LINE)
        {
            lineObejcts.append(object);
        }
        else if(object.getShapeType() == ShapeType::POLYGON)
        {
            polygonObejcts.append(object);
        }
    }
    this->drawRect.setRectList(rectObejcts);
    this->drawLine.setLineList(lineObejcts);
    this->drawPolygon.setPolygonList(polygonObejcts);
    this->sampleClass = sampleClass;
    drawPixmap();
}

QList<MyObject> EditableLabel::getObjects()
{
    QList<MyObject> allObject;
    QList<MyObject> rectObjects;
    QList<MyObject> lineObjects;
    QList<MyObject> polygonObjects;
    allObject.clear();

    rectObjects.clear();
    drawRect.getRectList(rectObjects);
    allObject.append(rectObjects);

    lineObjects.clear();
    drawLine.getLineList(lineObjects);
    allObject.append(lineObjects);

    polygonObjects.clear();
    drawPolygon.getPolygonList(polygonObjects);
    allObject.append(polygonObjects);

    return allObject;
}

void EditableLabel::drawPixmap()
{
    QPainter painter;
    tempPixmap = mp.copy();
    painter.begin(&tempPixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);

    drawRectPixmap(painter);
    drawLinePixmap(painter);
    drawPolygonPixmap(painter);

    painter.end();
    this->update();
}

void EditableLabel::drawRectPixmap(QPainter &painter)
{
    QPen pen(QColor("#3CFF55"), 2 ,Qt::DashLine);
    QFont font("Decorative", 15);
    painter.setPen(pen);
    painter.setFont(font);

    QList<MyObject> listObject;
    bool isDraw = false;
    QRect currentRect;
    QList<QPoint> points = drawRect.getRectListPoints(this->sampleClass);
    drawRect.getRectList(listObject);
    drawRect.getCurrentRect(currentRect, isDraw);

    for(int i = 0; i < listObject.size(); i++)
    {
        QString color = ManualParamterConfig::getMarkClassColor(listObject[i].getObjectClass());
        QColor drawColor(color);
        if(drawColor.isValid())
        {
            pen.setColor(drawColor);
            painter.setPen(pen);
        }
        else
        {
            drawColor = QColor("#000000");
            pen.setColor(drawColor);
            painter.setPen(pen);
        }
        if(this->sampleClass == "All")
        {
            painter.drawRect(listObject[i].getBox());
            painter.drawText(listObject[i].getBox().topLeft(), listObject[i].getObjectClass());
        }
        else
        {
            if(listObject[i].getObjectClass().contains(this->sampleClass))
            {
                painter.drawRect(listObject[i].getBox());
                painter.drawText(listObject[i].getBox().topLeft(), listObject[i].getObjectClass());
            }
        }
    }

    if(isDraw)
    {
        painter.drawRect(currentRect);
    }

    pen.setColor(QColor("#3CFF55"));
    painter.setPen(pen);
    painter.setBrush(QColor("#3CFF55"));
    for(int i = 0; i < points.size(); i++)
    {
        painter.drawEllipse(points[i], 2, 2);
    }
}

void EditableLabel::drawLinePixmap(QPainter &painter)
{
    QPen pen(QColor("#3CFF55"), 2 ,Qt::DashLine);
    QFont font("Decorative", 15);
    painter.setPen(pen);
    painter.setFont(font);

    QList<MyObject> listObject;
    bool isDraw = false;
    QPoint currentLine[2];
    drawLine.getCurrentLine(currentLine, isDraw);
    drawLine.getLineList(listObject);

    for(int i = 0; i < listObject.size(); i++)
    {
        QList<QPoint> line = listObject[i].getLine();
        QString color = ManualParamterConfig::getMarkClassColor(listObject[i].getObjectClass());
        QColor drawColor(color);
        if(drawColor.isValid())
        {
            pen.setColor(drawColor);
            painter.setPen(pen);
        }
        else
        {
            drawColor = QColor("#000000");
            pen.setColor(drawColor);
            painter.setPen(pen);
        }
        if(this->sampleClass == "All")
        {
            painter.drawLine(line[0], line[1]);
            painter.drawText(line[0], listObject[i].getObjectClass());

            painter.save();
            pen.setColor(QColor("#3CFF55"));
            painter.setPen(pen);
            painter.setBrush(QColor("#3CFF55"));
            painter.drawEllipse(line[0], 2, 2);
            painter.drawEllipse(line[1], 2, 2);
            painter.restore();
        }
        else
        {
            if(listObject[i].getObjectClass().contains(this->sampleClass))
            {
                painter.drawLine(line[0], line[1]);
                painter.drawText(line[0], listObject[i].getObjectClass());

                painter.save();
                pen.setColor(QColor("#3CFF55"));
                painter.setPen(pen);
                painter.setBrush(QColor("#3CFF55"));
                painter.drawEllipse(line[0], 2, 2);
                painter.drawEllipse(line[1], 2, 2);
                painter.restore();
            }
        }
    }

    if(isDraw)
    {
        painter.drawLine(currentLine[0], currentLine[1]);

        painter.save();
        pen.setColor(QColor("#3CFF55"));
        painter.setPen(pen);
        painter.setBrush(QColor("#3CFF55"));
        painter.drawEllipse(currentLine[0], 2, 2);
        painter.drawEllipse(currentLine[1], 2, 2);
        painter.restore();
    }
}

void EditableLabel::drawPolygonPixmap(QPainter &painter)
{
    QPen pen(QColor("#3CFF55"), 2 ,Qt::DashLine);
    QFont font("Decorative", 15);
    painter.setPen(pen);
    painter.setFont(font);
    painter.setBrush(QColor("#3CFF55"));

    QList<MyObject> listObject;
    bool isDraw = false;
    QPolygon currentPolygon = drawPolygon.getCurrentPolygon(isDraw);
    drawPolygon.getPolygonList(listObject);

    for(int i=0; i< listObject.count(); i++)
    {
        QString color = ManualParamterConfig::getMarkClassColor(listObject[i].getObjectClass());
        QColor drawColor(color);
        if(drawColor.isValid())
        {
            pen.setColor(drawColor);
            painter.setPen(pen);
        }
        else
        {
            drawColor = QColor("#000000");
            pen.setColor(drawColor);
            painter.setPen(pen);
        }
        if(this->sampleClass == "All")
        {
            QPolygon drawpoints = listObject[i].getPolygon();
            drawpoints.append(drawpoints.at(0));
            QPainterPath path;
            path.addPolygon(listObject[i].getPolygon());
            painter.fillPath(path, QBrush(QColor(drawColor.red(), drawColor.green(),
                                                drawColor.blue(), 80)));
            foreach (QPoint var, listObject[i].getPolygon())
            {
                painter.drawEllipse(var, 2, 2);
            }
            painter.drawPolyline(QPolygon(drawpoints));
            painter.drawText(drawpoints.at(0), listObject[i].getObjectClass());
        }
        else
        {
            if(listObject[i].getObjectClass().contains(this->sampleClass))
            {
                QPolygon drawpoints = listObject[i].getPolygon();
                drawpoints.append(drawpoints.at(0));
                QPainterPath path;
                path.addPolygon(listObject[i].getPolygon());
                painter.fillPath(path, QBrush(QColor(drawColor.red(),drawColor.green(),
                                                    drawColor.blue(), 80)));
                foreach (QPoint var, listObject[i].getPolygon())
                {
                    painter.drawEllipse(var, 2, 2);
                }
                painter.drawPolyline(QPolygon(drawpoints));
                painter.drawText(drawpoints.at(0), listObject[i].getObjectClass());
            }
        }
    }
    if(isDraw)
    {
        if(!currentPolygon.isEmpty())
        {
            painter.setBrush(QColor("#3CFF55"));
            foreach (QPoint var, currentPolygon)
            {
                painter.drawEllipse(var, 2, 2);
            }

            QPen pen(QColor("#3CFF55"), 2 ,Qt::DashLine);
            painter.setPen(pen);
            painter.drawPolyline(QPolygonF(currentPolygon));
        }
    }
}

void EditableLabel::initData()
{
    this->setMouseTracking(true);
    this->setCursor(Qt::CrossCursor);

    this->sampleClass = "All";

    this->shapeType = ShapeType::RECT;

    this->removeRectAction = new QAction(tr("删除标注"), this);
}

void EditableLabel::initConnect()
{
    connect(removeRectAction, &QAction::triggered, this, &EditableLabel::slotRemoveObject);
}
