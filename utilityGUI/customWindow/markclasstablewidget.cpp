#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "markclasstablewidget.h"
#include <QMenu>
#include <QContextMenuEvent>
#include "markclasswindow.h"
#include <iostream>

MarkClassTableWidget::MarkClassTableWidget(QWidget *parent): QTableWidget(parent)
{
    createActions();
    initConnect();
}

MarkClassTableWidget::~MarkClassTableWidget()
{

}

QMap<QString, QString> MarkClassTableWidget::getMarkClass()
{
    QMap<QString, QString> result;
    result.clear();
    for(int row = 0; row < this->rowCount(); row++)
    {
        result.insert(this->item(row, 0)->text(), this->item(row, 1)->text());
    }
    return result;
}

void MarkClassTableWidget::slotAdd()
{
    int row = this->currentRow();
    MarkClassWindow * window = new MarkClassWindow();
    window->setModal(true);
    window->setMarkClass(this->getMarkClass());
    int res = window->exec();
    if (res == QDialog::Accepted)
    {
        if(isAddEnd)
        {
            int index = this->rowCount();
            this->insertRow(index);
            QTableWidgetItem* tableItem0 = new QTableWidgetItem(window->getClassName());
            this->setItem(index, 0, tableItem0);
            QTableWidgetItem* tableItem1 = new QTableWidgetItem(window->getClassColor());
            tableItem1->setBackgroundColor(QColor(window->getClassColor()));
            this->setItem(index, 1, tableItem1);
            this->setCurrentItem(tableItem1);
        }
        else
        {
            int index = row + 1;
            this->insertRow(index);
            QTableWidgetItem* tableItem0 = new QTableWidgetItem(window->getClassName());
            this->setItem(index, 0, tableItem0);
            QTableWidgetItem* tableItem1 = new QTableWidgetItem(window->getClassColor());
            tableItem1->setBackgroundColor(QColor(window->getClassColor()));
            this->setItem(index, 1, tableItem1);
            this->setCurrentItem(tableItem1);
        }
    }
    window->deleteLater();
}

void MarkClassTableWidget::slotEdit()
{
    int row = this->currentRow();
    MarkClassWindow * window = new MarkClassWindow();
    window->setModal(true);
    window->setMarkClass(this->getMarkClass());
    window->setCalssName(this->item(row, 0)->text());
    window->setClassClolor(this->item(row, 1)->text());
    int res = window->exec();
    if (res == QDialog::Accepted)
    {
        QTableWidgetItem* tableItem0 = new QTableWidgetItem(window->getClassName());
        this->setItem(row, 0, tableItem0);
        QTableWidgetItem* tableItem1 = new QTableWidgetItem(window->getClassColor());
        tableItem1->setBackgroundColor(QColor(window->getClassColor()));
        this->setItem(row, 1, tableItem1);
        this->setCurrentItem(tableItem1);
    }
    window->deleteLater();
}

void MarkClassTableWidget::slotDelete()
{
    int row = this->currentRow();
    this->removeRow(row);
}

void MarkClassTableWidget::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu* popMenu = new QMenu(this);
    popMenu->clear(); //清除原有菜单
    QPoint point = event->pos(); //得到窗口坐标
    QTableWidgetItem *item = this->itemAt(point);
    if(item != NULL)
    {
        isAddEnd = false;
        popMenu->addAction(addAction);
        popMenu->addAction(editAction);
        popMenu->addAction(deleteAction);
        popMenu->addAction(refreshAction);

        popMenu->exec(QCursor::pos());
    }
    else
    {
        isAddEnd = true;
        popMenu->addAction(addAction);
        popMenu->addAction(refreshAction);
        popMenu->exec(QCursor::pos());
    }
}

void MarkClassTableWidget::createActions()
{
    addAction = new QAction(tr("新建"), this);
    editAction = new QAction(tr("编辑"), this);
    deleteAction = new QAction(tr("删除"), this);
    refreshAction = new QAction(tr("刷新"), this);
    refreshAction->setShortcut(QKeySequence::Refresh);
    isAddEnd = true;
}

void MarkClassTableWidget::initConnect()
{
    connect(addAction, &QAction::triggered, this, &MarkClassTableWidget::slotAdd);
    connect(editAction, &QAction::triggered, this, &MarkClassTableWidget::slotEdit);
    connect(deleteAction, &QAction::triggered, this, &MarkClassTableWidget::slotDelete);
}
