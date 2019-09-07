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
#include "drawShape/drawshape.h"

class SegmentLabel : public QLabel
{
public:
    SegmentLabel(QWidget *parent = 0);
    ~SegmentLabel();

    void clearObjects();
    void setNewQImage(QImage &image);
    void setOjects(const MyObject &object, const QString &sampleClass);
    QList<MyObject> getSegment();

public slots:

protected:
    void wheelEvent(QWheelEvent * event);
    void paintEvent(QPaintEvent *e);

private:

    void drawPixmap();

    void initData();
    void initConnect();

private:

    QPixmap mp;
    QPixmap tempPixmap;

    float scale;
    QImage *maskImage;
    QString sampleClass;
};

#endif // SEGMENTLABEL_H
