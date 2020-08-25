#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "modellabeltablewidget.h"
#include <QMenu>
#include <QContextMenuEvent>
#include "autoSampleMark/modellabelwindow.h"

ModelLabelTableWidget::ModelLabelTableWidget(QWidget *parent): QTableWidget(parent)
{
    createActions();
    initConnect();
}

ModelLabelTableWidget::~ModelLabelTableWidget()
{

}

QMap<int, QString> ModelLabelTableWidget::getModelLabels()
{
    QMap<int, QString> result;
    result.clear();
    for(int row = 0; row < this->rowCount(); row++)
    {
        result.insert(this->item(row, 0)->text().toInt(), this->item(row, 1)->text());
    }
    return result;
}

void ModelLabelTableWidget::slotAdd()
{
    int row = this->currentRow();
    ModelLabelWindow * window = new ModelLabelWindow();
    window->setModal(true);
    window->setModelLabel(this->getModelLabels());
    int res = window->exec();
    if (res == QDialog::Accepted)
    {
        this->insertRow(row);
        QTableWidgetItem* tableItem0 = new QTableWidgetItem(QString::number(window->getLabelId()));
        this->setItem(row, 0, tableItem0);
        QTableWidgetItem* tableItem1 = new QTableWidgetItem(window->getLabelName());
        this->setItem(row, 1, tableItem1);
        this->setCurrentItem(tableItem1);
    }
    window->deleteLater();
}

void ModelLabelTableWidget::slotEdit()
{
    int row = this->currentRow();
    ModelLabelWindow * window = new ModelLabelWindow();
    window->setModal(true);
    window->setModelLabel(this->getModelLabels());
    window->setLabelId(this->item(row, 0)->text().toInt());
    window->setLabelName(this->item(row, 1)->text());
    int res = window->exec();
    if (res == QDialog::Accepted)
    {
        QTableWidgetItem* tableItem0 = new QTableWidgetItem(QString::number(window->getLabelId()));
        this->setItem(row, 0, tableItem0);
        QTableWidgetItem* tableItem1 = new QTableWidgetItem(window->getLabelName());
        this->setItem(row, 1, tableItem1);
        this->setCurrentItem(tableItem1);
    }
    window->deleteLater();
}

void ModelLabelTableWidget::slotDelete()
{
    int row = this->currentRow();
    this->removeRow(row);
}

void ModelLabelTableWidget::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu* popMenu = new QMenu(this);
    popMenu->clear(); //清除原有菜单
    QPoint point = event->pos(); //得到窗口坐标
    QTableWidgetItem *item = this->itemAt(point);
    if(item != NULL)
    {
        popMenu->addAction(addAction);
        popMenu->addAction(editAction);
        popMenu->addAction(deleteAction);
        popMenu->addAction(refreshAction);

        popMenu->exec(QCursor::pos());
    }
}

void ModelLabelTableWidget::createActions()
{
    addAction = new QAction(tr("新建"), this);
    editAction = new QAction(tr("编辑"), this);
    deleteAction = new QAction(tr("删除"), this);
    refreshAction = new QAction(tr("刷新"), this);
    refreshAction->setShortcut(QKeySequence::Refresh);
}

void ModelLabelTableWidget::initConnect()
{
    connect(addAction, &QAction::triggered, this, &ModelLabelTableWidget::slotAdd);
    connect(editAction, &QAction::triggered, this, &ModelLabelTableWidget::slotEdit);
    connect(deleteAction, &QAction::triggered, this, &ModelLabelTableWidget::slotDelete);
}

