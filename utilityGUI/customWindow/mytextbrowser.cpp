#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "mytextbrowser.h"
#include <QMenu>

MyTextBrowser::MyTextBrowser(QWidget *parent):
    QTextBrowser(parent)
{
    initMenu();
    initConnect();
}

void MyTextBrowser::slotClearText()
{
    this->clear();
}

void MyTextBrowser::contextMenuEvent (QContextMenuEvent * event)
{
    QMenu* popMenu = new QMenu(this);
    popMenu->addAction(clearTextAction);
    // 菜单出现的位置为当前鼠标的位置
    popMenu->exec(QCursor::pos());
    QTextBrowser::contextMenuEvent(event);
}

void MyTextBrowser::initMenu()
{
    clearTextAction = new QAction(tr("清空信息"), this);
}

void MyTextBrowser::initConnect()
{
    connect(clearTextAction, &QAction::triggered, this, &MyTextBrowser::slotClearText);
}
