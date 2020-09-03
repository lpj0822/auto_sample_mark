﻿#ifndef MYSHAPE_H
#define MYSHAPE_H

#include <QObject>
#include <QString>
#include <QMap>

typedef enum ShapeType{
    UNSHAPE = -1,
    RECT_SHAPE = 0,
    LINE_SHAPE = 1,
    POLYGON_SHAPE = 2,
    POLYLINE_SHAPE = 3,
    POLYGON_SEGMENT_SHAPE = 4,
    LANE_SEGMENT_SHAPE = 5,
    MAX_IMAGE_SHAPE_TYPE = 6,
    //INSTANCE_SEGMENT_SHAPE
    RECT3D_SHAPE = 7,
    MAX_SHAPE_TYPE = 8
}ShapeType;

class MyShape
{
public:
    MyShape();
    ~MyShape();

    QString getShapeName(int shapeId);
    QMap<int, QString> getAllShape() const;
    QMap<int, QString> getImageShape() const;
    QMap<int, QString> getSegmentShape() const;

private:

    QMap<int, QString> shape;

private:

    void init();

};

#endif // MYSHAPE_H
