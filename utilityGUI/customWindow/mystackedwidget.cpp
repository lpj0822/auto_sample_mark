#include "mystackedwidget.h"

MyStackedWidget::MyStackedWidget(QWidget *parent): QStackedWidget(parent)
{

}

void MyStackedWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
    case Qt::Key_A:
        emit signalsKey(int(Qt::Key_A));
        break;
    case Qt::Key_D:
        emit signalsKey(int(Qt::Key_D));
        break;
    case Qt::Key_E:
        emit signalsKey(int(Qt::Key_E));
        break;
    case Qt::Key_J:
        emit signalsKey(int(Qt::Key_J));
        break;
    case Qt::Key_L:
        emit signalsKey(int(Qt::Key_L));
        break;
    case Qt::Key_Escape:
        emit signalsKey(int(Qt::Key_Escape));
        break;
    }
    if(event->modifiers() == Qt::ControlModifier)
    {
        if(event->key() == Qt::Key_Z)
        {
            emit signalsKey(int(Qt::Key_Z + Qt::ControlModifier));
        }
    }
}
