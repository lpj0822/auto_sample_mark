#ifndef MYSCROLLAREA_H
#define MYSCROLLAREA_H

#include <QWidget>
#include <QScrollArea>
#include <QKeyEvent>
#include <QString>

class MyScrollArea : public QScrollArea
{
    Q_OBJECT

public:
    MyScrollArea(QWidget *parent = 0);

signals:
    void signalsKey(int keyValue);

protected:
    void keyPressEvent(QKeyEvent *e);
};

#endif // MYSCROLLAREA_H
