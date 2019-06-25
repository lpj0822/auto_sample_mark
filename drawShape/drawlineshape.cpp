#pragma execution_character_set("utf-8")
#include "drawlineshape.h"
#include <QMessageBox>
#include <iostream>
#include <algorithm>
#include "sampleMarkParam/manualparamterconfig.h"
#include "selectmarkclasswindow.h"

DrawLineShape::DrawLineShape(QObject *parent) : QObject(parent)
{
    initDraw();
}

DrawLineShape::~DrawLineShape()
{

}

void DrawLineShape::initDraw()
{
    drawMousePressed = false;
    moveMousePressed = false;

    nearLineIndex = -1;
    linePointIndex = 0;
    removeLineIndex = -1;
    listLine.clear();
}

int DrawLineShape::drawLineMousePress(const QPoint point, bool &isDraw)
{
    int mouseChange = 0;
    nearLineIndex = nearLinePoint(point);
    if(nearLineIndex >= 0)
    {
        mouseChange = 2;
        drawMousePressed = false;
        moveMousePressed = true;
        updateLine(point);
    }
    else
    {
        mouseChange = 1;
        drawMousePressed = true;
        moveMousePressed = false;
        currentLine[0] = point;
        currentLine[1] = point;
    }
    isDraw = true;
    return mouseChange;
}

int DrawLineShape::drawLineMouseMove(const QPoint point, bool &isDraw)
{
    int mouseChange = 0;
    isDraw = false;
    if(drawMousePressed)
    {
        currentLine[1] = point;
        mouseChange = 1;
        isDraw = true;
    }
    else
    {
        if(moveMousePressed)
        {
            updateLine(point);
            mouseChange = 2;
            isDraw = true;
        }
        else
        {
            nearLineIndex = nearLinePoint(point);
            if(nearLineIndex >= 0)
            {
                mouseChange = 2;
            }
            else
            {
                mouseChange = 1;
            }
        }
    }
    return mouseChange;
}

int DrawLineShape::drawLineMouseRelease(QWidget *parent, const QPoint point, const QString sampleClass, bool &isDraw)
{
    if(drawMousePressed)
    {
        QPoint diffPoint = currentLine[0] - currentLine[1];

        if(diffPoint.manhattanLength() >= ManualParamterConfig::getMinWidth())
        {
            SelectMarkClassWindow *window = new SelectMarkClassWindow();
            window->setModal(true);
            window->setObjectRect(sampleClass);
            int res = window->exec();
            if (res == QDialog::Accepted)
            {
                MyObject object;
                object.setShapeType(ShapeType::LINE_SHAPE);
                object.setLine(currentLine[0], currentLine[1]);
                object.setObjectClass(window->getObjectClass());
                object.setObjectFlag(window->getObjectFlag());
                listLine.append(object);
            }
            window->deleteLater();
        }
        else
        {
            QMessageBox::information(parent, tr("标注"), tr("标注目标长度<%1").arg(ManualParamterConfig::getMinWidth()));
        }
    }
    else if(moveMousePressed)
    {
        QList<QPoint> line = listLine[nearLineIndex].getLine();
        QPoint diffPoint = line[0] - line[1];
        if(diffPoint.manhattanLength() >= ManualParamterConfig::getMinWidth())
        {

        }
        else
        {
            this->listLine.removeAt(nearLineIndex);
            QMessageBox::information(parent, tr("标注"), tr("标注目标长度<%1").arg(ManualParamterConfig::getMinWidth()));
        }
    }
    drawMousePressed = false;
    moveMousePressed = false;
    isDraw = true;
    return 0;
}

bool DrawLineShape::lineListContains(const QPoint point)
{
    bool isFind = false;
    removeLineIndex = listLine.size();
    for(int loop = 0; loop < listLine.size(); loop++)
    {
        QList<QPoint> line = listLine[loop].getLine();
        if(line.count() == 2)
        {
            int minX  = std::min(line[0].x(), line[1].x());
            int maxX = std::max(line[0].x(), line[1].x());
            int minY = std::min(line[0].y(), line[1].y());
            int maxY = std::max(line[0].y(), line[1].y());
            if(minX <= point.x() && maxX >= point.x()
                    && minY <= point.y() && maxY >= point.y())
            {
                float distance = geometryAlgorithm.pointToLineDistance(line[0], line[1], point);
                if(distance <= ManualParamterConfig::getNearPointLenght())
                {
                    removeLineIndex = loop;
                    isFind = true;
                    break;
                }
            }
        }
    }
    return isFind;
}

void DrawLineShape::removeLine(bool &isDraw)
{
    isDraw = false;
    if(removeLineIndex >= 0 && removeLineIndex < listLine.size())
    {
        this->listLine.removeAt(removeLineIndex);
        removeLineIndex = -1;
        isDraw = true;
    }
}

void DrawLineShape::getCurrentLine(QPoint *line, bool &isDraw)
{
    if(drawMousePressed)
    {
        isDraw = true;
    }
    else
    {
        isDraw = false;
    }
    line[0] = currentLine[0];
    line[1] = currentLine[1];
}

void DrawLineShape::getLineList(QList<MyObject> &list)
{
    list = this->listLine;
}

void DrawLineShape::setLineList(QList<MyObject> list)
{
    this->listLine.clear();
    this->listLine = list;
}

int DrawLineShape::nearLinePoint(const QPoint point)
{
    int resultIndex = -1;
    for(int index = 0; index < listLine.size(); index++)
    {
        QList<QPoint> line = listLine[index].getLine();
        QPoint point1 = point - line[0];
        if(point1.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            linePointIndex = 1;
            resultIndex = index;
            break;
        }
        QPoint point2 = point - line[1];
        if(point2.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            linePointIndex = 2;
            resultIndex = index;
            break;
        }
    }
    return resultIndex;
}

void DrawLineShape::updateLine(const QPoint point)
{
    if(linePointIndex  == 1)
    {
        QList<QPoint> line = listLine[nearLineIndex].getLine();
        line[0] = point;
        listLine[nearLineIndex].setLine(line[0], line[1]);
    }
    else if(linePointIndex  == 2)
    {
        QList<QPoint> line = listLine[nearLineIndex].getLine();
        line[1] = point;
        listLine[nearLineIndex].setLine(line[0], line[1]);
    }
}

