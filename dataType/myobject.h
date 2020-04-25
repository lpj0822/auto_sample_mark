#ifndef MYOBJECT_H
#define MYOBJECT_H

#include <QRect>
#include <QString>
#include <QList>
#include <QPolygon>
#include <QImage>
#include "myrect3d.h"
#include "drawShape/myshape.h"

class MyObject
{
public:
    MyObject();
    ~MyObject();

    void setShapeType(ShapeType type);
    ShapeType getShapeType() const;

    void setBox(QRect box);
    QRect getBox() const;

    void setBox3D(MyRect3D box);
    MyRect3D getBox3D() const;

    void setLine(QPoint startPoint, QPoint stopPoint);
    QList<QPoint> getLine() const;

    void setPolygon(QPolygon polygon);
    QPolygon getPolygon() const;

    void setPointList(QList<QPoint> &pointList);
    QList<QPoint> getPointList() const;

    void setObjectClass(QString objectClass);
    QString getObjectClass() const;

    void setLineWidth(const int width);
    int getLineWidth() const;

    void setIsDifficult(bool flag);
    bool getIsDifficult() const;

    void setObjectFlag(int flag);
    int getObjectFlag() const;

    void setIsTrackingObject(bool flag);
    bool getIsTrackingObject() const;

    void setSegmentImage(const QImage &image);
    QImage getSegmentImage() const;

private:
    int objectFlag;
    bool isDifficult;
    bool isTrackingObject;
    int lineWidth;
    MyRect3D box3D;
    QRect box;
    QList<QPoint> line;
    QList<QPoint> pointList;
    QPolygon polygon;
    QImage maskImage;
    QString obejctClass;
    ShapeType shapeType;

private:
    void init();
};

#endif // MYOBJECT_H
