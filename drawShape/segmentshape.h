#ifndef SEGMENTSHAPE_H
#define SEGMENTSHAPE_H

#include <QObject>
#include <QString>
#include <QMap>

typedef enum SegmentShapeType{
    UNSHAPE = -1,
    POLYGON_SHAPE = 0,
    MAX_SHAPE_TYPE = 1
}SegmentShapeType;

class SegmentShape
{
public:
    SegmentShape();
    ~SegmentShape();

    QString getShapeName(int shapeId);
    QMap<int, QString> getAllShape() const;

private:

    QMap<int, QString> shape;

private:

    void init();
};

#endif // SEGMENTSHAPE_H
