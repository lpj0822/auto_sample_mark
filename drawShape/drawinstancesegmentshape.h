#ifndef DRAWINSTANCESEGMENTSHAPE_H
#define DRAWINSTANCESEGMENTSHAPE_H

#include "drawShape/drawshape.h"

class DrawInstanceSegmentShape : public DrawShape
{
    Q_OBJECT
public:
    DrawInstanceSegmentShape(MarkDataType dataType, QObject *parent = 0);
    ~DrawInstanceSegmentShape();

    void initDraw();

    //rect
    int drawMousePress(const QPoint point, bool &isDraw);
    int drawMouseMove(const QPoint point, bool &isDraw);
    int drawMouseRelease(QWidget *parent, const QPoint point, bool &isDraw);
    void removeShape(bool &isDraw);
    bool isInShape(const QPoint &point);

    void drawPixmap(const ShapeType shapeID, QPainter &painter);

    void setObjectList(QList<MyObject> list);
    void getObjectList(QList<MyObject> &list);

    void getCurrentRect(QRect &rect, bool &isDraw);
    QList<QPoint> getRectListPoints(const QString sampleClass);

signals:

public slots:

private:

    int nearRectPiont(const QPoint point);
    void updateRect(const QPoint point);

private:
    int perScale;
    int scale;

    int nearRectIndex;
    int rectPointIndex;
    int removeRectIndex;
    QRect currentRect;
    QList<MyObject> listRect;
};

#endif // DRAWINSTANCESEGMENTSHAPE_H
