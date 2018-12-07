#ifndef DRAWLINESHAPE_H
#define DRAWLINESHAPE_H

#include <QObject>
#include "myobject.h"
#include "baseAlgorithm/geometryalgorithm.h"

class DrawLineShape : public QObject
{
    Q_OBJECT
public:
    DrawLineShape(QObject *parent = 0);
    ~DrawLineShape();

    void initDraw();

    //line
    int drawLineMousePress(const QPoint point, bool &isDraw);
    int drawLineMouseMove(const QPoint point, bool &isDraw);
    int drawLineMouseRelease(QWidget *parent, const QPoint point, const QString sampleClass, bool &isDraw);

    bool lineListContains(const QPoint point);
    void removeLine(bool &isDraw);

    void getCurrentLine(QPoint *line, bool &isDraw);
    void getLineList(QList<MyObject> &list);
    void setLineList(QList<MyObject> list);

signals:

public slots:

private:

    int nearLinePoint(const QPoint point);
    void updateLine(const QPoint point);

private:

    bool drawMousePressed;
    bool moveMousePressed;

    int nearLineIndex;
    int linePointIndex;
    int removeLineIndex;
    QPoint currentLine[2];
    QList<MyObject> listLine;

    GeometryAlgorithm geometryAlgorithm;
};

#endif // DRAWLINESHAPE_H
