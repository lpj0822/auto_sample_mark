#pragma execution_character_set("utf-8")
#include "manualparamterconfigwindow.h"
#include <QColorDialog>
#include <QMessageBox>
#include <limits>

ManualParamterConfigWindow::ManualParamterConfigWindow(QDialog *parent) : QDialog(parent), paramterConfig()
{
    init();
    initUI();
    initConnect();
}

ManualParamterConfigWindow::~ManualParamterConfigWindow()
{

}

void ManualParamterConfigWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存手动标注参数配置信息"), tr("是否保存配置信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        slotOk();
    }
    QDialog::closeEvent(event);
}

void ManualParamterConfigWindow::slotOk()
{
    paramterConfig.setMinWidth(this->minWidthBox->value());
    paramterConfig.setMinHeight(this->minHeightBox->value());
    paramterConfig.setMarkClass(this->markClassTable->getMarkClass());
    paramterConfig.saveConfig();
    this->accept();
}

void ManualParamterConfigWindow::slotClassModelSelect(QString text)
{
    int index = this->classModelSelectBox->currentData().toInt();
    classModelWidget->setCurrentIndex(index);
}

void ManualParamterConfigWindow::init()
{
    paramterConfig.loadConfig();
}

void ManualParamterConfigWindow::initUI()
{
    classModelSelectLabel = new QLabel(tr("目标属性模式选择："));
    classModelSelectBox = new QComboBox();
    initModelSelect();
    QHBoxLayout *modelLayout = new QHBoxLayout();
    modelLayout->setSpacing(10);
    modelLayout->addWidget(classModelSelectLabel);
    modelLayout->addWidget(classModelSelectBox);

    minWidthLabel = new QLabel(tr("最小可标注宽度"));
    minWidthBox = new QSpinBox();
    minWidthBox->setSingleStep(10);
    minWidthBox->setMinimum(0);
    minWidthBox->setMaximum(INT_MAX);
    minWidthBox->setValue(paramterConfig.getMinWidth());

    minHeightLabel = new QLabel(tr("最小可标注高度"));
    minHeightBox = new QSpinBox();
    minHeightBox->setSingleStep(10);
    minHeightBox->setMinimum(0);
    minHeightBox->setMaximum(INT_MAX);
    minHeightBox->setValue(paramterConfig.getMinHeight());

    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->setSpacing(10);
    topLayout->addWidget(minWidthLabel);
    topLayout->addWidget(minWidthBox);
    topLayout->addWidget(minHeightLabel);
    topLayout->addWidget(minHeightBox);

    classModelWidget = new QStackedWidget(this);
    markClassTable = new MarkClassTableWidget();
    initTable();
    markClassTree = new MarkClassTreeWidget();
    initTree();
    classModelWidget->addWidget(markClassTable);
    classModelWidget->addWidget(markClassTree);
    classModelWidget->setCurrentIndex(0);

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
    mainLayout->addLayout(modelLayout);
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(classModelWidget);
    mainLayout->addLayout(bottomLayout);

    this->setLayout(mainLayout);
    this->setMaximumSize(420, 500);
    this->setMinimumSize(420, 500);
    this->setWindowTitle(tr("手动标注参数配置"));
}

void ManualParamterConfigWindow::initConnect()
{
    connect(loadDefaultButton, &QPushButton::clicked, this, &ManualParamterConfigWindow::loadDefaultValue);
    connect(saveButton, &QPushButton::clicked, this, &ManualParamterConfigWindow::slotOk);
    connect(cancelButton, &QPushButton::clicked, this, &ManualParamterConfigWindow::reject);
    connect(classModelSelectBox, &QComboBox::currentTextChanged, this, &ManualParamterConfigWindow::slotClassModelSelect);
}

void ManualParamterConfigWindow::initModelSelect()
{
    classModelSelectBox->addItem(tr("单级属性模式"), 0);
    classModelSelectBox->addItem(tr("多级属性模式"), 1);
}

void ManualParamterConfigWindow::initTable()
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

void ManualParamterConfigWindow::initTree()
{
    markClassTree->clear();
    markClassTree->setHeaderLabel(tr("类别属性添加"));
}

void ManualParamterConfigWindow::loadDefaultValue()
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
    minWidthBox->setValue(20);
    minHeightBox->setValue(20);
}
