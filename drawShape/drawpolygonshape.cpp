#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "drawpolygonshape.h"
#include <QMessageBox>
#include "sampleMarkParam/manualparamterconfig.h"
#include "selectmarkclasswindow.h"

DrawPolygonShape::DrawPolygonShape(MarkDataType dataType, bool isSegment, QObject *parent) :
    DrawShape(dataType, parent), isSegment(isSegment)
{
    initDraw();
}

DrawPolygonShape::~DrawPolygonShape()
{

}

void DrawPolygonShape::initDraw()
{
    drawMousePressed = false;
    moveMousePressed = false;

    finishDrawPolygon = false;
    nearFirstPoint = false;
    firstPoint = QPoint(-1, -1);

    nearPolygonIndex = -1;
    polygonPointIndex = 0;
    removePolygonIndex = -1;
    listPolygon.clear();
}

int DrawPolygonShape::drawMousePress(const QPoint point, bool &isDraw)
{
    int mouseChange = 0;
    if(nearPolygonIndex >= 0)
    {
        mouseChange = 2;
        drawMousePressed = false;
        moveMousePressed = true;
        updatePolygon(point);
    }
    else
    {
        if(firstPoint.x() < 0 || firstPoint.y() < 0)
        {
            firstPoint = point;
        }
        if(!nearFirstPoint)
        {
            currentPolygon.append(point);
        }
        else
        {
            finishDrawPolygon = true;
        }
        mouseChange = 1;
        moveMousePressed = false;
        drawMousePressed = true;
    }
    isDraw = true;
    return mouseChange;
}

int DrawPolygonShape::drawMouseMove(const QPoint point, bool &isDraw)
{
    int mouseChange = 0;
    isDraw = false;
    if(moveMousePressed)
    {
        updatePolygon(point);
        mouseChange = 2;
        isDraw = true;
    }
    else if(!drawMousePressed)
    {
        nearPolygonIndex = nearPolygonPoint(point);
        if(nearPolygonIndex >= 0 || nearFirstPoint)
        {
            mouseChange = 2;
        }
        else
        {
            mouseChange = 1;
        }
    }
    return mouseChange;
}

int DrawPolygonShape::drawMouseRelease(QWidget *parent, const QPoint point, bool &isDraw)
{
    if(finishDrawPolygon)
    {
        if(currentPolygon.count() > 2)
        {
            float area = geometryAlgorithm.polygonArea(currentPolygon);
            int minArea = ManualParamterConfig::getMinWidth() * ManualParamterConfig::getMinHeight();
            if(area >= minArea)
            {
                SelectMarkClassWindow *window = new SelectMarkClassWindow(this->markDataType);
                window->setModal(true);
                window->setObjectRect(this->visibleSampleClass);
                int res = window->exec();
                if (res == QDialog::Accepted)
                {
                    MyObject object;
                    if(this->markDataType == MarkDataType::SEGMENT)
                    {
                        object.setShapeType(ShapeType::SEGMENT_POLYGON_SHAPE);
                    }
                    else
                    {
                        object.setShapeType(ShapeType::POLYGON_SHAPE);
                    }
                    object.setPolygon(currentPolygon);
                    object.setObjectClass(window->getObjectClass());
                    object.setIsDifficult(window->getIsDifficult());
                    object.setObjectFlag(window->getObjectFlag());
                    listPolygon.append(object);
                }
                window->deleteLater();
            }
            else
            {
                QMessageBox::information(parent, tr("标注"), tr("标注目标面积小于%1").arg(minArea));
            }
        }
        else
        {
            QMessageBox::information(parent, tr("标注"), tr("标注目标有误!"));
        }
        currentPolygon.clear();
        firstPoint = QPoint(-1, -1);
        finishDrawPolygon = false;
    }
    else if(moveMousePressed)
    {
        QPolygon polygon = listPolygon[nearPolygonIndex].getPolygon();
        int pointCount = polygon.count();
        int preIndex = ((polygonPointIndex-1) - 1 + pointCount) % pointCount;
        int nextIndex = ((polygonPointIndex-1) + 1) % pointCount;
        QPoint prePoint = polygon[preIndex];
        QPoint nextPoint = polygon[nextIndex];
        QPoint movePoint = polygon[polygonPointIndex-1];
        QPoint point1 = movePoint - prePoint;
        QPoint point2 = movePoint - nextPoint;
        if(point1.manhattanLength() <= ManualParamterConfig::getNearPointLenght() ||
                point2.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            polygon.removeAt(polygonPointIndex-1);
            if(polygon.count() <= 2)
            {
                this->listPolygon.removeAt(nearPolygonIndex);
                QMessageBox::information(parent, tr("标注"), tr("标注目标有误!"));
            }
            else
            {
                listPolygon[nearPolygonIndex].setPolygon(polygon);
            }
        }
    }
    drawMousePressed = false;
    moveMousePressed = false;
    isDraw = true;
    return 0;
}

void DrawPolygonShape::removeShape(bool &isDraw)
{
    isDraw = false;
    if(removePolygonIndex >= 0 && removePolygonIndex < listPolygon.count())
    {
        this->listPolygon.removeAt(removePolygonIndex);
        removePolygonIndex = -1;
        isDraw = true;
    }
}

bool DrawPolygonShape::isInShape(const QPoint &point)
{
    bool isFind = false;
    removePolygonIndex = listPolygon.count();
    for(int loop = 0; loop < listPolygon.count(); loop++)
    {
        QPolygon polygon = listPolygon[loop].getPolygon();
        if(polygon.containsPoint(point, Qt::OddEvenFill))
        {
            removePolygonIndex = loop;
            isFind = true;
            break;
        }
    }
    return isFind;
}

void DrawPolygonShape::drawPixmap(const ShapeType shapeID, QPainter &painter)
{
    int drawLineWidth = 2;
    if(isSegment)
    {
        drawLineWidth = 1;
    }
    QPen pen(QColor("#3CFF55"), drawLineWidth ,Qt::DashLine);
    QFont font("Decorative", 15);
    painter.setPen(pen);
    painter.setFont(font);
    painter.setBrush(QColor("#3CFF55"));

    bool isDraw = false;
    QPolygon currentPolygon = getCurrentPolygon(isDraw);

    for(int i=0; i< this->listPolygon.count(); i++)
    {
        QString color = ManualParamterConfig::getMarkClassColor(this->listPolygon[i].getObjectClass());
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
        if(this->visibleSampleClass == "All")
        {
            QPolygon drawpoints = this->listPolygon[i].getPolygon();
            drawpoints.append(drawpoints.at(0));
            QPainterPath path;
            path.addPolygon(this->listPolygon[i].getPolygon());
            painter.fillPath(path, QBrush(QColor(drawColor.red(), drawColor.green(),
                                                drawColor.blue(), 80)));
            foreach (QPoint var, this->listPolygon[i].getPolygon())
            {
                painter.drawEllipse(var, 2, 2);
            }
            painter.drawPolyline(QPolygon(drawpoints));
            painter.drawText(drawpoints.at(0), this->listPolygon[i].getObjectClass());
        }
        else
        {
            if(this->listPolygon[i].getObjectClass().contains(this->visibleSampleClass))
            {
                QPolygon drawpoints = this->listPolygon[i].getPolygon();
                drawpoints.append(drawpoints.at(0));
                QPainterPath path;
                path.addPolygon(this->listPolygon[i].getPolygon());
                painter.fillPath(path, QBrush(QColor(drawColor.red(),drawColor.green(),
                                                    drawColor.blue(), 80)));
                foreach (QPoint var, this->listPolygon[i].getPolygon())
                {
                    painter.drawEllipse(var, 2, 2);
                }
                painter.drawPolyline(QPolygon(drawpoints));
                painter.drawText(drawpoints.at(0), this->listPolygon[i].getObjectClass());
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
            painter.setBrush(QColor("#000000"));
            painter.drawEllipse(firstPoint, 4, 4);
            QPen pen(QColor("#3CFF55"), drawLineWidth ,Qt::DashLine);
            painter.setPen(pen);
            painter.drawPolyline(QPolygonF(currentPolygon));
        }
    }
}

void DrawPolygonShape::setObjectList(QList<MyObject> list)
{
    this->listPolygon.clear();
    this->listPolygon = list;
}

void DrawPolygonShape::getObjectList(QList<MyObject> &list)
{
    list = this->listPolygon;
}

int DrawPolygonShape::getObjectSize()
{
    return this->listPolygon.count();
}

void DrawPolygonShape::createImageMask(QImage &maskImage)
{
    for(int i=0; i< this->listPolygon.count(); i++)
    {
        drawImageMask.drawPolygonMaskImage(this->listPolygon[i], this->visibleSampleClass, maskImage);
    }
}

QPolygon DrawPolygonShape::getCurrentPolygon(bool &isDraw)
{
    isDraw = true;
    return this->currentPolygon;
}

int DrawPolygonShape::nearPolygonPoint(const QPoint point)
{
    int resultIndex = -1;
    int moveCount = listPolygon.size();
    if(firstPoint.x() >= 0 && firstPoint.y() >= 0)
    {
        QPoint point1 = point - firstPoint;
        if(point1.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            nearFirstPoint = true;
        }
        else
        {
            nearFirstPoint = false;
        }
    }
    else
    {
        nearFirstPoint = false;
    }
    for(int index = 0; index < moveCount; index++)
    {
        QPolygon polygon = listPolygon[index].getPolygon();
        for(int pointIndex = 0; pointIndex < polygon.count(); pointIndex++)
        {
           QPoint point1 = point - polygon[pointIndex];
           if(point1.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
           {
               polygonPointIndex = pointIndex + 1;
               resultIndex = index;
               return resultIndex;
           }
        }
    }
    return resultIndex;
}

void DrawPolygonShape::updatePolygon(const QPoint point)
{
    QPolygon polygon = listPolygon[nearPolygonIndex].getPolygon();
    polygon[polygonPointIndex - 1] = point;
    listPolygon[nearPolygonIndex].setPolygon(polygon);
}
