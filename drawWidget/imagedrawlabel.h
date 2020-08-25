#ifndef IMAGEDRAWLABEL_H
#define IMAGEDRAWLABEL_H

#include <QLabel>
#include <QPainter>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDateTime>
#include <QToolButton>
#include <QPoint>
#include <QAction>
#include <QWheelEvent>

#include "drawShape/drawshape.h"

class ImageDrawLabel : public QLabel
{
public:
    ImageDrawLabel(QWidget *parent = 0);
    virtual ~ImageDrawLabel();

    virtual void clearDraw() = 0;
    virtual void setNewQImage(QImage &image) = 0;
    virtual void setDrawShape(int shapeID) = 0;
    virtual void resetDraw() = 0;

    virtual void setOjects(QList<MyObject> obejcts, QString sampleClass);
    virtual void setMaskOject(const MyObject &mask);
    virtual QList<MyObject> getObjects();

    virtual void undoDrawShape();

public slots:

    void slotRemoveObject();

protected:

    virtual void setDrawShapeObjects() = 0;
    virtual void drawPixmap() = 0;

    void init();

protected:

    QAction *removeRectAction;

    QPixmap mp;
    QPixmap tempPixmap;

    ShapeType shapeType;
    QMap<ShapeType, DrawShape*> drawList;

    QString sampleClass;

    QList<MyObject> rectObejcts;
    QList<MyObject> lineObejcts;
    QList<MyObject> polygonObejcts;
    QList<MyObject> laneObejcts;

    QList<MyObject> polygonSegObejcts;
    QList<MyObject> laneSegObejcts;
};

#endif // IMAGEDRAWLABEL_H
