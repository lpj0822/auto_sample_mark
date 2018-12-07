#include "myscrollarea.h"

MyScrollArea::MyScrollArea(QWidget *parent): QScrollArea(parent)
{

}

void MyScrollArea::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_A)
    {
        emit signalsKey(int(Qt::Key_A));
    }
    else if(e->key() == Qt::Key_D)
    {
        emit signalsKey(int(Qt::Key_D));
    }
    else if(e->key() == Qt::Key_Escape)
    {
        emit signalsKey(int(Qt::Key_Escape));
    }
    else if(e->key() == Qt::Key_J)
    {
        emit signalsKey(int(Qt::Key_J));
    }
    else if(e->key() == Qt::Key_L)
    {
        emit signalsKey(int(Qt::Key_L));
    }
    else if(e->key() == Qt::Key_E)
    {
        emit signalsKey(int(Qt::Key_E));
    }
}
