#ifndef SEGMENTLABEL_H
#define SEGMENTLABEL_H

#include <QLabel>
#include <QPainter>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDateTime>
#include <QToolButton>
#include <QPoint>
#include <QList>
#include <QAction>
#include <QWheelEvent>
#include <QList>
#include "drawShape/drawpolygonshape.h"
#include "drawShape/drawlaneshape.h"
#include "saveMarkData/segmentimagesave.h"

class SegmentLabel : public QLabel
{
public:
    SegmentLabel(QWidget *parent = 0);
    ~SegmentLabel();

    void clearObjects();
    void setNewQImage(QImage &image);
    void setDrawShape(int shapeID);
    void setOjects(const MyObject &mask, const QList<MyObject> &obejcts, const QString &sampleClass);
    QList<MyObject> getObjects();
    MyObject getSegmentMask();

public slots:

    void slotRemoveObject();

protected:
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void wheelEvent(QWheelEvent * event);

    void paintEvent(QPaintEvent *e);

    void contextMenuEvent(QContextMenuEvent * event);

private:

    void drawPixmap();

    void drawSegmentMask(QPainter &painter);

    void setMaskOject(const MyObject &mask);

    void initData();
    void initConnect();

private:

    QAction *removeRectAction;

    QPixmap mp;
    QPixmap tempPixmap;

    float scale;
    QImage *maskImage;
    QString sampleClass;

    ShapeType shapeType;
    QMap<ShapeType, DrawShape*> drawList;

    SegmentImageSave segmentPorcess;
};

#endif // SEGMENTLABEL_H
