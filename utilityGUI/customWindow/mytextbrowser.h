#ifndef MYTEXTBROWSER_H
#define MYTEXTBROWSER_H

#include <QWidget>
#include <QTextBrowser>
#include <QContextMenuEvent>
#include <QAction>

class MyTextBrowser : public QTextBrowser
{
public:
    MyTextBrowser(QWidget *parent = 0);

public slots:

    void slotClearText();

protected:
    void contextMenuEvent (QContextMenuEvent * event);

private:
     void initMenu();
     void initConnect();

private:
     QAction *clearTextAction;
};

#endif // MYTEXTBROWSER_H
