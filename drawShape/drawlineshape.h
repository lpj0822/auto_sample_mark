#ifndef DRAWLINESHAPE_H
#define DRAWLINESHAPE_H

#include "drawshape.h"

class DrawLineShape : public DrawShape
{
    Q_OBJECT
public:
    DrawLineShape(MarkDataType dataType, QObject *parent = 0);
    ~DrawLineShape();

    void initDraw();

    int drawMousePress(const QPoint point, bool &isDraw);
    int drawMouseMove(const QPoint point, bool &isDraw);
    int drawMouseRelease(QWidget *parent, const QPoint point, bool &isDraw);
    void removeShape(bool &isDraw);
    bool isInShape(const QPoint &point);

    void drawPixmap(const QString &sampleClass, const ShapeType shapeID, QPainter &painter);

    void setObjectList(QList<MyObject> list);
    void getObjectList(QList<MyObject> &list);

    void getCurrentLine(QPoint *line, bool &isDraw);

signals:

public slots:

private:

    int nearLinePoint(const QPoint point);
    void updateLine(const QPoint point);

private:

    int nearLineIndex;
    int linePointIndex;
    int removeLineIndex;
    QPoint currentLine[2];
    QList<MyObject> listLine;
};

#endif // DRAWLINESHAPE_H
