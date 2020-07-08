#ifndef EDITABLELABEL_H
#define EDITABLELABEL_H

#include <QLabel>
#include <QPainter>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDateTime>
#include <QToolButton>
#include <QPoint>
#include <QAction>
#include <QWheelEvent>
#include <QList>
#include <QMap>

#include "drawShape/drawrectshape.h"
#include "drawShape/drawlineshape.h"
#include "drawShape/drawpolygonshape.h"
#include "drawShape/drawlaneshape.h"
#include "dataType/mark_data_type.h"

class EditableLabel : public QLabel
{
public:
    EditableLabel(QWidget *parent = 0);
    ~EditableLabel();

    void clearObjects();
    void setNewQImage(QImage &image);
    void setDrawShape(int shapeID);
    void setOjects(QList<MyObject> obejcts, QString sampleClass);
    QList<MyObject> getObjects();

public slots:

    void slotRemoveObject();

protected:
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent * event);

    void paintEvent(QPaintEvent *e);

    void contextMenuEvent(QContextMenuEvent * event);

private:

    void drawPixmap();

    QPointF offsetToCenter();
    QPoint scalePoint(const QPoint point);

    void initData();
    void initConnect();

private:

    QAction *removeRectAction;

    QString sampleClass;

    QPixmap mp;
    QPixmap tempPixmap;

    ShapeType shapeType;
    QMap<ShapeType, DrawShape*> drawList;

    int zoomValue;
};

#endif // EDITABLELABEL_H
