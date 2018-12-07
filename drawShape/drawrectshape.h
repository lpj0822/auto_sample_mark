#ifndef DRAWRECTSHAPE_H
#define DRAWRECTSHAPE_H

#include <QObject>
#include "myobject.h"

class DrawRectShape : public QObject
{
    Q_OBJECT
public:
    DrawRectShape(QObject *parent = 0);
    ~DrawRectShape();

    void initDraw();

    //rect
    int drawRectMousePress(const QPoint point, bool &isDraw);
    int drawRectMouseMove(const QPoint point, bool &isDraw);
    int drawRectMouseRelease(QWidget *parent, const QPoint point, const QString sampleClass, bool &isDraw);

    bool rectListContains(const QPoint point);
    void removeRect(bool &isDraw);

    void getCurrentRect(QRect &rect, bool &isDraw);
    void getRectList(QList<MyObject> &list);
    QList<QPoint> getRectListPoints(const QString sampleClass);
    void setRectList(QList<MyObject> list);

signals:

public slots:

private:

    int nearRectPiont(const QPoint point);
    void updateRect(const QPoint point);

private:

    bool drawMousePressed;
    bool moveMousePressed;

    int perScale;
    int scale;

    int nearRectIndex;
    int rectPointIndex;
    int removeRectIndex;
    QRect currentRect;
    QList<MyObject> listRect;
};

#endif // DRAWRECTSHAPE_H
