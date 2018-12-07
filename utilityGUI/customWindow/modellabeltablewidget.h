#ifndef MODELLABELTABLEWIDGET_H
#define MODELLABELTABLEWIDGET_H

#include <QWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QAction>

class ModelLabelTableWidget: public QTableWidget
{
    Q_OBJECT

public:
    ModelLabelTableWidget(QWidget *parent = 0);
    ~ModelLabelTableWidget();

    QMap<int, QString> getModelLabels();


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

#endif // MODELLABELTABLEWIDGET_H
