#ifndef MYOBJECT_H
#define MYOBJECT_H

#include <QRect>
#include <QString>
#include <QList>
#include <QPolygon>
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

    void setLine(QPoint startPoint, QPoint stopPoint);
    QList<QPoint> getLine() const;

    void setPolygon(QPolygon polygon);
    QPolygon getPolygon() const;

    void setObjectClass(QString objectClass);
    QString getObjectClass() const;

    void setObjectFlag(int flag);
    int getObjectFlag() const;

    void setIsTrackingObject(bool flag);
    bool getIsTrackingObject() const;

private:
    int objectFlag;
    bool isTrackingObject;
    QRect box;
    QList<QPoint> line;
    QPolygon polygon;
    QString obejctClass;
    ShapeType shapeType;

private:
    void init();
};

#endif // MYOBJECT_H
