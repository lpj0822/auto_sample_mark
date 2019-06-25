#pragma execution_character_set("utf-8")
#include "drawrectshape.h"
#include <QMessageBox>
#include "sampleMarkParam/manualparamterconfig.h"
#include "selectmarkclasswindow.h"

DrawRectShape::DrawRectShape(QObject *parent) : QObject(parent)
{
    initDraw();
}

DrawRectShape::~DrawRectShape()
{

}

void DrawRectShape::initDraw()
{
    drawMousePressed = false;
    moveMousePressed = false;

    scale = 100;
    perScale = 100;

    nearRectIndex = -1;
    rectPointIndex = 0;
    removeRectIndex = -1;
    listRect.clear();
}

int DrawRectShape::drawRectMousePress(const QPoint point, bool &isDraw)
{
    int mouseChange = 0;
    nearRectIndex = nearRectPiont(point);
    if(nearRectIndex >= 0)
    {
        mouseChange = 2;
        drawMousePressed = false;
        moveMousePressed = true;
        updateRect(point);
    }
    else
    {
        mouseChange = 1;
        drawMousePressed = true;
        moveMousePressed = false;
        currentRect.setTopLeft(point);
        currentRect.setBottomRight(point);
    }
    isDraw = true;
    return mouseChange;
}

int DrawRectShape::drawRectMouseMove(const QPoint point, bool &isDraw)
{
    int mouseChange = 0;
    isDraw = false;
    if(drawMousePressed)
    {
        currentRect.setBottomRight(point);
        mouseChange = 1;
        isDraw = true;
    }
    else
    {
        if(moveMousePressed)
        {
            updateRect(point);
            mouseChange = 2;
            isDraw = true;
        }
        else
        {
            nearRectIndex = nearRectPiont(point);
            if(nearRectIndex >= 0)
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

int DrawRectShape::drawRectMouseRelease(QWidget *parent, const QPoint point, const QString sampleClass, bool &isDraw)
{
    if(drawMousePressed)
    {
        int rectMinX = std::min(currentRect.topLeft().x(), currentRect.bottomRight().x());
        int rectMinY = std::min(currentRect.topLeft().y(), currentRect.bottomRight().y());
        int rectMaxX = std::max(currentRect.topLeft().x(), currentRect.bottomRight().x());
        int rectMaxY = std::max(currentRect.topLeft().y(), currentRect.bottomRight().y());
        currentRect.setTopLeft(QPoint(rectMinX, rectMinY));
        currentRect.setBottomRight(QPoint(rectMaxX, rectMaxY));

        if(currentRect.width() >= ManualParamterConfig::getMinWidth()
                && currentRect.height() >= ManualParamterConfig::getMinHeight())
        {
            SelectMarkClassWindow *window = new SelectMarkClassWindow();
            window->setModal(true);
            window->setObjectRect(sampleClass);
            int res = window->exec();
            if (res == QDialog::Accepted)
            {
                MyObject object;
                object.setShapeType(ShapeType::RECT_SHAPE);
                object.setBox(currentRect);
                object.setObjectClass(window->getObjectClass());
                object.setObjectFlag(window->getObjectFlag());
                listRect.append(object);
            }
            window->deleteLater();
        }
        else
        {
            QMessageBox::information(parent, tr("标注"), tr("标注目标宽度<%1或者高度<%2").arg(
                                         ManualParamterConfig::getMinWidth()).arg(ManualParamterConfig::getMinHeight()));
        }
    }
    else if(moveMousePressed)
    {
        QRect rect = listRect[nearRectIndex].getBox();
        int minWidth = ManualParamterConfig::getMinWidth() * scale / 100.0f;
        int minHeight = ManualParamterConfig::getMinHeight() * scale / 100.0f;
        if(rect.width() >= minWidth && rect.height() >= minHeight)
        {

        }
        else
        {
            this->listRect.removeAt(nearRectIndex);
            QMessageBox::information(parent, tr("标注"), tr("标注目标宽度<%1或者高度<%2").arg(minWidth).arg(minHeight));
        }
    }
    drawMousePressed = false;
    moveMousePressed = false;
    isDraw = true;
    return 0;
}

bool DrawRectShape::rectListContains(const QPoint point)
{
    bool isFind = false;
    removeRectIndex = listRect.size();
    for(int loop = 0; loop < listRect.size(); loop++)
    {
        QRect rect = listRect[loop].getBox();
        if(rect.contains(point))
        {
            removeRectIndex = loop;
            isFind = true;
            break;
        }
    }
    return isFind;
}

void DrawRectShape::removeRect(bool &isDraw)
{
    isDraw = false;
    if(removeRectIndex >= 0 && removeRectIndex < listRect.size())
    {
        this->listRect.removeAt(removeRectIndex);
        removeRectIndex = -1;
        isDraw = true;
    }
}

void DrawRectShape::getCurrentRect(QRect &rect, bool &isDraw)
{
    if(drawMousePressed)
    {
        isDraw = true;
    }
    else
    {
        isDraw = false;
    }
    rect = this->currentRect;
}

void DrawRectShape::getRectList(QList<MyObject> &list)
{
    list = this->listRect;
}

QList<QPoint> DrawRectShape::getRectListPoints(const QString sampleClass)
{
    QList<QPoint> pointList;
    pointList.clear();
    for(int index = 0; index < listRect.size(); index++)
    {
        if(sampleClass == "All")
        {
            QRect box = listRect[index].getBox();
            pointList.append(box.topLeft());
            pointList.append(box.topRight());
            pointList.append(box.bottomLeft());
            pointList.append(box.bottomRight());
        }
        else if(listRect[index].getObjectClass().contains(sampleClass))
        {
            QRect box = listRect[index].getBox();
            pointList.append(box.topLeft());
            pointList.append(box.topRight());
            pointList.append(box.bottomLeft());
            pointList.append(box.bottomRight());
        }
    }
    if(drawMousePressed)
    {
        pointList.append(currentRect.topLeft());
        pointList.append(currentRect.topRight());
        pointList.append(currentRect.bottomLeft());
        pointList.append(currentRect.bottomRight());
    }
    return pointList;
}

void DrawRectShape::setRectList(QList<MyObject> list)
{
    this->listRect.clear();
    this->listRect = list;
}

int DrawRectShape::nearRectPiont(const QPoint point)
{
    int resultIndex = -1;
    for(int index = 0; index < listRect.size(); index++)
    {
        QRect box = listRect[index].getBox();
        QPoint point1 = point - box.topLeft();
        if(point1.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            rectPointIndex = 1;
            resultIndex = index;
            break;
        }
        QPoint point2 = point - box.topRight();
        if(point2.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            rectPointIndex = 2;
            resultIndex = index;
            break;
        }
        QPoint point3 = point - box.bottomLeft();
        if(point3.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            rectPointIndex = 3;
            resultIndex = index;
            break;
        }
        QPoint point4 = point - box.bottomRight();
        if(point4.manhattanLength() <= ManualParamterConfig::getNearPointLenght())
        {
            rectPointIndex = 4;
            resultIndex = index;
            break;
        }
    }
    return resultIndex;
}

void DrawRectShape::updateRect(const QPoint point)
{
    if(rectPointIndex  == 1)
    {
        QRect rect = listRect[nearRectIndex].getBox();
        rect.setTopLeft(point);
        listRect[nearRectIndex].setBox(rect);
        listRect[nearRectIndex].setIsTrackingObject(false);
    }
    else if(rectPointIndex  == 2)
    {
        QRect rect = listRect[nearRectIndex].getBox();
        rect.setTopRight(point);
        listRect[nearRectIndex].setBox(rect);
        listRect[nearRectIndex].setIsTrackingObject(false);
    }
    else if(rectPointIndex  == 3)
    {
        QRect rect = listRect[nearRectIndex].getBox();
        rect.setBottomLeft(point);
        listRect[nearRectIndex].setBox(rect);
        listRect[nearRectIndex].setIsTrackingObject(false);
    }
    else if(rectPointIndex  == 4)
    {
        QRect rect = listRect[nearRectIndex].getBox();
        rect.setBottomRight(point);
        listRect[nearRectIndex].setBox(rect);
        listRect[nearRectIndex].setIsTrackingObject(false);
    }
}
