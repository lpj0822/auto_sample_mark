#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "drawlaneshape.h"
#include <QMessageBox>
#include "sampleMarkParam/manualparamterconfig.h"
#include "sampleMarkParam/segmentparamterconfig.h"
#include "selectmarkclasswindow.h"

DrawLaneShape::DrawLaneShape(MarkDataType dataType, bool isSegment,
                             bool isCurveFit, QObject *parent) :
    DrawShape(dataType, parent), isSegment(isSegment), maskImage(NULL), isCurveFit(isCurveFit)
{
    initDraw();
}

DrawLaneShape::~DrawLaneShape()
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
}

void DrawLaneShape::initDraw()
{
    drawMousePressed = false;
    moveMousePressed = false;

    nearFirstPoint = false;
    firstPoint = QPoint(-1, -1);

    nearPolygonIndex = -1;
    polygonPointIndex = 0;
    removePolygonIndex = -1;
    listLane.clear();
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
    this->laneWidth = 10;
}

void DrawLaneShape::setLaneWidth(const int laneWidth)
{
    this->laneWidth = laneWidth;
}

int DrawLaneShape::drawMousePress(const QPoint point, bool &isDraw)
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
        mouseChange = 1;
        moveMousePressed = false;
        drawMousePressed = true;
    }
    isDraw = true;
    return mouseChange;
}

int DrawLaneShape::drawMouseMove(const QPoint point, bool &isDraw)
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

int DrawLaneShape::drawMouseRelease(QWidget *parent, const QPoint point, bool &isDraw)
{
    if(moveMousePressed)
    {
        QList<QPoint> polygon = listLane[nearPolygonIndex].getPointList();
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
            if(polygon.count() < 3)
            {
                this->listLane.removeAt(nearPolygonIndex);
                QMessageBox::information(parent, tr("标注"), tr("标注目标有误!"));
            }
            else
            {
                listLane[nearPolygonIndex].setPointList(polygon);
            }
        }
    }
    drawMousePressed = false;
    moveMousePressed = false;
    isDraw = true;
    return 0;
}

int DrawLaneShape::drawMouseDoubleClick(QWidget *parent, const QPoint point, bool &isDraw)
{
    if(currentPolygon.count() >= 3)
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
                object.setShapeType(ShapeType::LANE_SHAPE);
            }
            else
            {
                object.setShapeType(ShapeType::POLYLINE_SHAPE);
            }
            object.setLineWidth(this->laneWidth);
            QList<QPoint> temp = currentPolygon.toList();
            object.setPointList(temp);
            object.setObjectClass(window->getObjectClass());
            object.setIsDifficult(window->getIsDifficult());
            object.setObjectFlag(window->getObjectFlag());
            listLane.append(object);
        }
        window->deleteLater();
    }
    else
    {
        QMessageBox::information(parent, tr("标注"), tr("标注目标点数需要大于3!"));
    }
    currentPolygon.clear();
    firstPoint = QPoint(-1, -1);
    drawMousePressed = false;
    moveMousePressed = false;
    isDraw = true;
    return 0;
}

void DrawLaneShape::removeShape(bool &isDraw)
{
    isDraw = false;
    if(removePolygonIndex >= 0 && removePolygonIndex < listLane.count())
    {
        this->listLane.removeAt(removePolygonIndex);
        removePolygonIndex = -1;
        isDraw = true;
    }
}

bool DrawLaneShape::isInShape(const QPoint &point)
{
    bool isFind = false;
    removePolygonIndex = listLane.count();
    for(int loop = 0; loop < listLane.count(); loop++)
    {
        QList<QPoint> polygon = listLane[loop].getPointList();
        int index = 0;
        for(; index < polygon.count(); index++)
        {
            QPoint diffPoint = polygon[index] - point;
            if(diffPoint.manhattanLength() < 20)
            {
                removePolygonIndex = loop;
                isFind = true;
                break;
            }
        }
        if(index < polygon.count())
        {
            break;
        }
    }
    return isFind;
}

void DrawLaneShape::drawPixmap(const QString &sampleClass, const ShapeType shapeID, QPainter &painter)
{
    const int height = painter.device()->height();
    const int width = painter.device()->width();
    int drawLineWidth = 2;
    if(isSegment)
    {
        drawLineWidth = 1;
    }
    QPen pen(QColor("#3CFF55"), drawLineWidth ,Qt::DashLine);
    QFont font("Decorative", 10);
    painter.setPen(pen);
    painter.setFont(font);
    painter.setBrush(QColor("#3CFF55"));

    bool isDraw = false;
    QPolygon currentPolygon = getCurrentPolygon(isDraw);

    if(isSegment)
    {
        this->laneWidth = SegmentParamterConfig::getLineWidth();
        if(maskImage != NULL)
        {
            delete maskImage;
            maskImage = NULL;
        }
        maskImage = new QImage(width, height, QImage::Format_ARGB32);
        maskImage->fill(0);
    }

    if(shapeID == ShapeType::LANE_SHAPE)
    {
        painter.save();
        QPen tempPen(QColor(0, 0, 0, 120));
        tempPen.setWidth(drawLineWidth);
        painter.setPen(tempPen);
        for(int loop = 10; loop < height; loop += 10)
        {
            painter.drawLine(QPoint(0, loop), QPoint(width, loop));
        }
        painter.restore();
    }

    for(int i=0; i< this->listLane.count(); i++)
    {
        QString color = ManualParamterConfig::getMarkClassColor(this->listLane[i].getObjectClass());
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
        if(sampleClass == "All")
        {
            if(this->listLane[i].getLineWidth() > 0)
            {
                this->laneWidth = this->listLane[i].getLineWidth();
            }
            drawMark(this->listLane[i].getPointList(), this->laneWidth, pen, painter);
            painter.drawText(this->listLane[i].getPointList().at(0), this->listLane[i].getObjectClass());
        }
        else
        {
            if(this->listLane[i].getObjectClass().contains(sampleClass))
            {
                if(this->listLane[i].getLineWidth() > 0)
                {
                    this->laneWidth = this->listLane[i].getLineWidth();
                }
                drawMark(this->listLane[i].getPointList(), this->laneWidth, pen, painter);
                painter.drawText(this->listLane[i].getPointList().at(0), this->listLane[i].getObjectClass());
            }
        }
    }
    if(isSegment)
    {
        if(this->listLane.count() > 0 && maskImage != NULL)
        {
            painter.save();
            painter.setOpacity(0.5);
            painter.drawImage(QPoint(0, 0), *maskImage);
            painter.restore();
        }
    }
    if(isDraw)
    {
        if(!currentPolygon.isEmpty())
        {
            painter.setBrush(QColor("#3CFF55"));
            foreach(QPoint var, currentPolygon)
            {
                painter.drawEllipse(var, 2, 2);
            }

            QPen pen(QColor("#3CFF55"), drawLineWidth ,Qt::DashLine);
            painter.setPen(pen);
            painter.drawPolyline(QPolygonF(currentPolygon));
        }
    }
}

void DrawLaneShape::setObjectList(QList<MyObject> list)
{
    this->listLane.clear();
    this->listLane = list;
}

void DrawLaneShape::getObjectList(QList<MyObject> &list)
{
    list = this->listLane;
}

int DrawLaneShape::getObjectSize()
{
    return this->listLane.count();
}

void DrawLaneShape::setSegmentImage(const MyObject &object)
{
    if(maskImage != NULL)
    {
        delete maskImage;
        maskImage = NULL;
    }
    maskImage = new QImage(object.getSegmentImage());
}

MyObject DrawLaneShape::getSegmentImage()
{
    MyObject result;
    if(maskImage != NULL)
        result.setSegmentImage(*maskImage);
    return result;
}

void DrawLaneShape::getCurveFitPointList(const QList<QPoint> &inputKeyPoint, QList<QPoint> &result)
{
    cv::Mat A;
    int minX = 0;
    int maxX = 0;
    result.clear();
    if(inputKeyPoint.size() > 0)
    {
        curveFit.polynomialCurveFit(inputKeyPoint, 3, A);
        if(inputKeyPoint[0].x() > inputKeyPoint[inputKeyPoint.size() - 1].x())
        {
            minX = inputKeyPoint[inputKeyPoint.size() - 1].x();
            maxX = inputKeyPoint[0].x();
        }
        else
        {
            minX = inputKeyPoint[0].x();
            maxX = inputKeyPoint[inputKeyPoint.size() - 1].x();
        }
    }
    for (double x = minX; x < maxX; x += 0.2)
    {
        double y = A.at<double>(0, 0) + A.at<double>(1, 0) * x +
                A.at<double>(2, 0)*std::pow(x, 2) + A.at<double>(3, 0)*std::pow(x, 3);
        int tempX = static_cast<int>(std::round(x));
        int tempY = static_cast<int>(std::round(y));
        result.append(QPoint(tempX, tempY));
    }
}

QPolygon DrawLaneShape::getCurrentPolygon(bool &isDraw)
{
    isDraw = true;
    return this->currentPolygon;
}

int DrawLaneShape::nearPolygonPoint(const QPoint point)
{
    int resultIndex = -1;
    int moveCount = listLane.size();
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
        QList<QPoint> polygon = listLane[index].getPointList();
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

void DrawLaneShape::updatePolygon(const QPoint point)
{
    QList<QPoint> polygon = listLane[nearPolygonIndex].getPointList();
    polygon[polygonPointIndex - 1] = point;
    listLane[nearPolygonIndex].setPointList(polygon);
}

void DrawLaneShape::drawMark(const QList<QPoint> &pointList, const int width,
                             QPen &pen, QPainter &painter)
{
    QPolygon drawpoints;
    QList<QPoint> resultPoints;
    for(int loop = 0; loop < pointList.size(); loop++)
    {
        drawpoints.append(pointList[loop]);
        painter.drawEllipse(pointList[loop], 2, 2);
    }
    painter.drawPolyline(QPolygon(drawpoints));
    if(isCurveFit)
    {
        getCurveFitPointList(pointList, resultPoints);
        painter.save();
        painter.setPen(QPen(QColor(0, 0, 0)));
        for(int loop = 0; loop < resultPoints.size(); loop++)
        {
            painter.drawEllipse(resultPoints[loop], 1, 1);
        }
        painter.restore();
    }
    else
    {
        resultPoints = pointList;
    }
    if(isSegment)
    {
        drawMaskImage(resultPoints, width, pen.color());
    }
}

void DrawLaneShape::drawMaskImage(const QList<QPoint> &pointList, const int width,
                                  const QColor &color)
{
    int harf_width = width / 2;
    if(maskImage != NULL)
    {
        QPainter painter;
        QPen pen(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QBrush brush(color, Qt::SolidPattern);
        painter.begin(maskImage);
        painter.setPen(pen);
        painter.setBrush(brush);
        for(int loop = 0; loop < pointList.count(); loop++)
        {
            int y = std::min(pointList[loop].y(), maskImage->height());
            int minX = std::max(0, pointList[loop].x() - harf_width);
            int maxX = std::min(pointList[loop].x() + harf_width, maskImage->width());
            painter.drawLine(QPoint(minX, y), QPoint(maxX, y));
        }
        painter.end();
    }
}
