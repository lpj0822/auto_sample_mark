#pragma execution_character_set("utf-8")
#include "markclasstreewidget.h"
#include <QMenu>
#include <QContextMenuEvent>
#include <QInputDialog>
#include <QMessageBox>
#include <QLineEdit>
#include "markclasswindow.h"

MarkClassTreeWidget::MarkClassTreeWidget(QWidget *parent): QTreeWidget(parent)
{
    createActions();
    initConnect();
    classNameList.clear();
}

MarkClassTreeWidget::~MarkClassTreeWidget()
{

}

void MarkClassTreeWidget::slotAdd()
{
    QString inputName = QInputDialog::getText(this,tr("添加类别"),tr("类别名字(英文)"));
    QString className = inputName.trimmed();
    if(!className.isEmpty() && !classNameList.contains(className))
    {
        QTreeWidgetItem *tempItem = new QTreeWidgetItem(QStringList(className));
        this->addTopLevelItem(tempItem);
        classNameList.append(className);
    }
    else
    {
        QMessageBox::information(this, tr("标注类别参数"), tr("%1标注类别存在").arg(className));
    }
}

void MarkClassTreeWidget::slotAddSub()
{
    QTreeWidgetItem *tempItem = this->currentItem();
    if(tempItem != NULL)
    {
        QString inputName = QInputDialog::getText(this,tr("添加类别"),tr("类别名字(英文)"));
        QString className = inputName.trimmed();
        if(!className.isEmpty() && !classNameList.contains(className))
        {
            QTreeWidgetItem *childItem = new QTreeWidgetItem(tempItem, QStringList(className));
            tempItem->addChild(childItem);
            classNameList.append(className);
        }
        else
        {
            QMessageBox::information(this, tr("标注类别参数"), tr("%1标注类别存在").arg(className));
        }
    }
}

void MarkClassTreeWidget::slotEdit()
{
    QTreeWidgetItem *tempItem = this->currentItem();
    int column = this->currentColumn();
    QString text = tempItem->text(column);
    if(tempItem != NULL)
    {
        QString inputName = QInputDialog::getText(this, tr("修改类别"), tr("类别名字(英文)"), QLineEdit::Normal, text);
        QString className = inputName.trimmed();
        if(!className.isEmpty() && !classNameList.contains(className))
        {
            tempItem->setText(column, className);
            classNameList.removeOne(text);
            classNameList.append(className);
        }
        else
        {
            QMessageBox::information(this, tr("标注类别参数"), tr("%1标注类别存在").arg(className));
        }
    }
}

void MarkClassTreeWidget::slotDelete()
{
    QTreeWidgetItem *tempItem = this->currentItem();
    int column = this->currentColumn();
    this->removeItemWidget(tempItem, column);
}

void MarkClassTreeWidget::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu* popMenu = new QMenu(this);
    popMenu->clear(); //清除原有菜单
    QPoint point = event->pos(); //得到窗口坐标
    QTreeWidgetItem *item = this->itemAt(point);
    if(item != NULL)
    {
        popMenu->addAction(addAction);
        popMenu->addAction(addSubAction);
        popMenu->addAction(editAction);
        popMenu->addAction(deleteAction);
        popMenu->addAction(refreshAction);

        popMenu->exec(QCursor::pos());
    }
    else
    {
        popMenu->addAction(addAction);
        popMenu->addAction(refreshAction);

        popMenu->exec(QCursor::pos());
    }
}

void MarkClassTreeWidget::createActions()
{
    addAction = new QAction(tr("新建类别"), this);
    addSubAction = new QAction(tr("添加子类别"), this);
    editAction = new QAction(tr("编辑类别"), this);
    deleteAction = new QAction(tr("删除类别"), this);
    refreshAction = new QAction(tr("刷新列表"), this);
    refreshAction->setShortcut(QKeySequence::Refresh);
}

void MarkClassTreeWidget::initConnect()
{
    connect(addAction, &QAction::triggered, this, &MarkClassTreeWidget::slotAdd);
    connect(addSubAction, &QAction::triggered, this, &MarkClassTreeWidget::slotAddSub);
    connect(editAction, &QAction::triggered, this, &MarkClassTreeWidget::slotEdit);
    connect(deleteAction, &QAction::triggered, this, &MarkClassTreeWidget::slotDelete);
}
