#ifndef MYSHAPE_H
#define MYSHAPE_H

#include <QObject>
#include <QString>
#include <QMap>

typedef enum ShapeType{
    UNSHAPE = -1,
    RECT = 0,
    LINE = 1,
    POLYGON = 2
}ShapeType;

class MyShape
{
public:
    MyShape();
    ~MyShape();

    QString getShapeName(int shapeId);
    QMap<int, QString> getAllShape() const;

private:

    QMap<int, QString> shape;

private:

    void init();

};

#endif // MYSHAPE_H
