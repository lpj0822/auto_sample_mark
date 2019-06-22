#ifndef MYSTACKEDWIDGET_H
#define MYSTACKEDWIDGET_H

#include <QWidget>
#include <QStackedWidget>
#include <QKeyEvent>
#include <QString>

class MyStackedWidget : public QStackedWidget
{
    Q_OBJECT
public:
    MyStackedWidget(QWidget *parent = 0);
    ~MyStackedWidget() = default;

signals:
    void signalsKey(int keyValue);

protected:
    void keyPressEvent(QKeyEvent *e);
};

#endif // MYSTACKEDWIDGET_H
