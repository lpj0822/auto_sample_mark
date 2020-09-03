#ifndef SEGMENTLABEL_H
#define SEGMENTLABEL_H

#include "drawWidget/imagedrawlabel.h"

#include <QList>
#include <QCursor>

#include "drawShape/drawpolygonshape.h"
#include "drawShape/drawlaneshape.h"

class SegmentLabel : public ImageDrawLabel
{
public:
    SegmentLabel(QWidget *parent = 0);
    ~SegmentLabel();

    void clearDraw() override;
    void setNewQImage(QImage &image) override;
    void setDrawShape(int shapeID) override;
    void resetDraw() override;

    void setMaskOject(const MyObject &mask) override;

public slots:

protected:
    void mouseMoveEvent(QMouseEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent * event) override;

    void paintEvent(QPaintEvent *e) override;

    void contextMenuEvent(QContextMenuEvent * event) override;

    void setDrawShapeObjects() override;
    void drawPixmap() override;

private:

    void drawSegmentMask(QPainter &painter);

    QPointF offsetToCenter();
    QPoint scalePoint(const QPoint point);

    void initData();

private:
    float scale;
    QImage *maskImage;
    QCursor myDrawCursor;
};

#endif // SEGMENTLABEL_H
