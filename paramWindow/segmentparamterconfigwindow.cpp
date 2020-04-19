#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
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
    paramterConfig.setLineWidth(lineWidthBox->value());
    paramterConfig.saveConfig();
    this->accept();
}

void SegmentParamterConfigWindow::init()
{
    paramterConfig.loadConfig();
}

void SegmentParamterConfigWindow::initUI()
{
    lineWidthLabel = new QLabel(tr("线条区域宽度"));
    lineWidthBox = new QSpinBox();
    lineWidthBox->setMinimum(1);
    lineWidthBox->setMaximum(200);
    lineWidthBox->setSingleStep(1);
    lineWidthBox->setValue(paramterConfig.getLineWidth());

    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->setSpacing(30);
    topLayout->addWidget(lineWidthLabel);
    topLayout->addWidget(lineWidthBox);

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
    mainLayout->addLayout(topLayout);
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

void SegmentParamterConfigWindow::loadDefaultValue()
{
    lineWidthBox->setValue(10);
}
