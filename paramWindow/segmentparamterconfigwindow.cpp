#pragma execution_character_set("utf-8")
#include "segmentparamterconfigwindow.h"
#include <QColorDialog>
#include <QMessageBox>
#include <limits>

SegmentParamterConfigWindow::SegmentParamterConfigWindow(QDialog *parent) :
    QDialog(parent), paramterConfig()
{
    init();
    initUI();
    initConnect();
}

SegmentParamterConfigWindow::~SegmentParamterConfigWindow()
{

}

void SegmentParamterConfigWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存分割标注参数配置信息"), tr("是否保存配置信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        slotOk();
    }
}

void SegmentParamterConfigWindow::slotOk()
{
    paramterConfig.setMarkClass(this->markClassTable->getMarkClass());
    paramterConfig.saveConfig();
    this->accept();
}

void SegmentParamterConfigWindow::init()
{
    paramterConfig.loadConfig();
}

void SegmentParamterConfigWindow::initUI()
{
    markClassTable = new MarkClassTableWidget();
    initTable();

    loadDefaultButton = new QPushButton(tr("恢复默认值"));
    saveButton = new QPushButton(tr("保存"));
    cancelButton = new QPushButton(tr("取消"));

    QHBoxLayout *bottomLayout = new QHBoxLayout();
    bottomLayout->setSpacing(30);
    bottomLayout->setAlignment(Qt::AlignRight);
    bottomLayout->addWidget(loadDefaultButton);
    bottomLayout->addWidget(saveButton);
    bottomLayout->addWidget(cancelButton);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setSpacing(10);
    mainLayout->addWidget(markClassTable);
    mainLayout->addLayout(bottomLayout);

    this->setLayout(mainLayout);
    this->setMaximumSize(420, 500);
    this->setMinimumSize(420, 500);
    this->setWindowTitle(tr("分割标注参数配置"));
}

void SegmentParamterConfigWindow::initConnect()
{
    connect(loadDefaultButton, &QPushButton::clicked, this, &SegmentParamterConfigWindow::loadDefaultValue);
    connect(saveButton, &QPushButton::clicked, this, &SegmentParamterConfigWindow::slotOk);
    connect(cancelButton, &QPushButton::clicked, this, &SegmentParamterConfigWindow::reject);
}

void SegmentParamterConfigWindow::initTable()
{
    int row = 0;
    QMap<QString, QString>::const_iterator classIterator;
    QStringList headerName;
    QMap<QString, QString> markClassData = paramterConfig.getMarkClass();
    int rows = markClassData.count();
    markClassTable->clear();
    markClassTable->setRowCount(rows);
    markClassTable->setColumnCount(2);
    headerName << QString(tr("类别")) << QString(tr("颜色"));
    markClassTable->setHorizontalHeaderLabels(headerName);
    markClassTable->horizontalHeader()->setEnabled(false);
    markClassTable->verticalHeader()->setVisible(false);
    markClassTable->horizontalHeader()->setStretchLastSection(true);
    markClassTable->horizontalHeader()->resizeSection(0,150); //设置表头第一列的宽度为150
    markClassTable->horizontalHeader()->setFixedHeight(25); //设置表头的高度
    markClassTable->setSelectionBehavior(QTableWidget::SelectRows);
    markClassTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    markClassTable->setFrameShape(QFrame::NoFrame); //设置无边框
    markClassTable->setSelectionMode(QAbstractItemView::SingleSelection);
    for(classIterator = markClassData.constBegin(); classIterator != markClassData.constEnd(); ++classIterator)
    {
        QTableWidgetItem* tableItem0 = new QTableWidgetItem(classIterator.key());
        markClassTable->setItem(row, 0, tableItem0);
        QTableWidgetItem* tableItem1 = new QTableWidgetItem(classIterator.value());
        tableItem1->setBackgroundColor(QColor(classIterator.value()));
        markClassTable->setItem(row, 1, tableItem1);
        row++;
    }
}

void SegmentParamterConfigWindow::loadDefaultValue()
{
    int row = 0;
    QMap<QString, QString>::const_iterator classIterator;
    QMap<QString, QString> markClassData;
    QStringList headerName;
    markClassData.clear();
    markClassData.insert("person", "#FF0000");
    markClassData.insert("car", "#00FF00");
    markClassData.insert("bus", "#0000FF");
    markClassData.insert("bicycle", "#FFFF00");
    markClassData.insert("truck", "#FF00FF");
    markClassData.insert("motorbike", "#00FFFF");
    markClassData.insert("background", "#FFFFFF");
    int rows = markClassData.count();
    markClassTable->clear();
    markClassTable->setRowCount(rows);
    markClassTable->setColumnCount(2);
    headerName << QString(tr("类别")) << QString(tr("颜色"));
    markClassTable->setHorizontalHeaderLabels(headerName);
    for(classIterator = markClassData.constBegin(); classIterator != markClassData.constEnd(); ++classIterator)
    {
        QTableWidgetItem* tableItem0 = new QTableWidgetItem(classIterator.key());
        markClassTable->setItem(row, 0, tableItem0);
        QTableWidgetItem* tableItem1 = new QTableWidgetItem(classIterator.value());
        tableItem1->setBackgroundColor(QColor(classIterator.value()));
        markClassTable->setItem(row, 1, tableItem1);
        row++;
    }
}
