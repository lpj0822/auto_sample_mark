#ifndef DRAWSHAPE_H
#define DRAWSHAPE_H

#include <QObject>
#include <QPainter>
#include "dataType/myobject.h"
#include "dataType/mark_data_type.h"
#include "baseAlgorithm/geometryalgorithm.h"

class DrawShape : public QObject
{
    Q_OBJECT
public:
    DrawShape(MarkDataType dataType, QObject *parent = nullptr);

    virtual void initDraw() = 0;
    virtual int drawMousePress(const QPoint point, bool &isDraw) = 0;
    virtual int drawMouseMove(const QPoint point, bool &isDraw) = 0;
    virtual int drawMouseRelease(QWidget *parent, const QPoint point, bool &isDraw) = 0;
    virtual void removeShape(bool &isDraw) = 0;
    virtual bool isInShape(const QPoint &point) = 0;

    virtual void drawPixmap(const ShapeType shapeID, QPainter &painter) = 0;

    virtual void setObjectList(QList<MyObject> list) = 0;
    virtual void getObjectList(QList<MyObject> &list) = 0;

    virtual int getObjectSize();

    virtual int drawMouseDoubleClick(QWidget *parent, const QPoint point, bool &isDraw);

    virtual void createImageMask(QImage &maskImage);

    void setVisibleSampleClass(const QString &sampleClass);

protected:

    bool drawMousePressed;
    bool moveMousePressed;
    GeometryAlgorithm geometryAlgorithm;
    MarkDataType markDataType;

    QString visibleSampleClass;
};

#endif // DRAWSHAPE_H
