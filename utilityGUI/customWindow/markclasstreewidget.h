#ifndef MYTREEWIDGET_H
#define MYTREEWIDGET_H

#include <QWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QAction>

class MarkClassTreeWidget : public QTreeWidget
{
    Q_OBJECT

public:
    MarkClassTreeWidget(QWidget *parent = 0);
    ~MarkClassTreeWidget();


signals:

public slots:
    void slotAdd();
    void slotAddSub();
    void slotEdit();
    void slotDelete();

protected:
    void contextMenuEvent(QContextMenuEvent *event);

private:
    QAction *addAction;
    QAction *addSubAction;
    QAction *editAction;
    QAction *deleteAction;
    QAction *refreshAction;

    QList<QString> classNameList;

private:
    void createActions();
    void initConnect();
};

#endif // MYTREEWIDGET_H
