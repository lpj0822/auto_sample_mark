#ifndef EDITABLELABEL_H
#define EDITABLELABEL_H

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

#include "drawShape/drawrectshape.h"
#include "drawShape/drawlineshape.h"
#include "drawShape/drawpolygonshape.h"
#include "drawShape/drawlaneshape.h"

class EditableLabel : public QLabel
{
public:
    EditableLabel(QWidget *parent = 0);

    void clearObjects();
    void setNewQImage(QImage &image);
    void setDrawShape(int shapeID);
    void setOjects(QList<MyObject> obejcts, QString sampleClass);
    QList<MyObject> getObjects();
    QList<MyObject> getSegment();

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

    void initData();
    void initConnect();

private:

    QAction *removeRectAction;

    QString sampleClass;

    ShapeType shapeType;

    QPixmap mp;
    QPixmap tempPixmap;

    QList<DrawShape*> drawList;
};

#endif // EDITABLELABEL_H
