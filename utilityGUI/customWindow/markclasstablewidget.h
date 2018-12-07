#ifndef MYTABLEWIDGET_H
#define MYTABLEWIDGET_H

#include <QWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QAction>

class MarkClassTableWidget : public QTableWidget
{
    Q_OBJECT

public:
    MarkClassTableWidget(QWidget *parent = 0);
    ~MarkClassTableWidget();

    QMap<QString, QString> getMarkClass();


signals:

public slots:
    void slotAdd();
    void slotEdit();
    void slotDelete();

protected:
    void contextMenuEvent(QContextMenuEvent *event);

private:
    QAction *addAction;
    QAction *editAction;
    QAction *deleteAction;
    QAction *refreshAction;

private:
    void createActions();
    void initConnect();
};

#endif // MYTABLEWIDGET_H
