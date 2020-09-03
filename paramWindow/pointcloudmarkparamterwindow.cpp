#pragma execution_character_set("utf-8")
#include "pointcloudmarkparamterwindow.h"
#include <QMessageBox>
#include <limits>

PointCloudMarkParamterWindow::PointCloudMarkParamterWindow(QDialog *parent) : QDialog(parent), paramterConfig()
{
    init();
    initUI();
    initConnect();
}

PointCloudMarkParamterWindow::~PointCloudMarkParamterWindow()
{

}

void PointCloudMarkParamterWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存点云标注参数配置信息"), tr("是否保存配置信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        slotOk();
    }
    QDialog::closeEvent(event);
}

void PointCloudMarkParamterWindow::slotOk()
{
    paramterConfig.setFieldsNumber(this->fieldsNumberBox->value());
    paramterConfig.setFileType(this->formatBox->currentData().toInt());
    paramterConfig.saveConfig();
    this->accept();
}

void PointCloudMarkParamterWindow::loadDefaultValue()
{
    this->fieldsNumberBox->setValue(3);
    this->formatBox->setCurrentIndex(0);
}

void PointCloudMarkParamterWindow::init()
{
    paramterConfig.loadConfig();
}

void PointCloudMarkParamterWindow::initUI()
{
    QHBoxLayout *layout = new QHBoxLayout();
    layout->setSpacing(30);
    formatLabel = new QLabel(tr("读取文件格式:"));
    formatBox = new QComboBox();
    initFileType();
    layout->addWidget(formatLabel);
    layout->addWidget(formatBox);

    fieldsNumberLabel = new QLabel(tr("读取字段数:"));
    fieldsNumberBox = new QSpinBox();
    fieldsNumberBox->setMinimum(3);
    fieldsNumberBox->setMaximum(4);
    fieldsNumberBox->setValue(paramterConfig.getFieldsNumber());
    fieldsNumberBox->setSingleStep(1);

    QHBoxLayout *layout1 = new QHBoxLayout();
    layout1->setSpacing(30);
    layout1->addWidget(fieldsNumberLabel);
    layout1->addWidget(fieldsNumberBox);

    infoLabel = new QLabel(tr("注意：\n读取文件格式目前只支持pcd与bin文件，\n"
                              "其中读取字段数参数只与读bin文件有关，\n"
                              "该参数描述了一个包含几个字段信息"));

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
    mainLayout->addLayout(layout);
    mainLayout->addLayout(layout1);
    mainLayout->addWidget(infoLabel);
    mainLayout->addLayout(bottomLayout);

    this->setLayout(mainLayout);
    this->setMaximumSize(420, 250);
    this->setMinimumSize(420, 250);
    this->setWindowTitle(tr("点云标注参数配置"));
}

void PointCloudMarkParamterWindow::initConnect()
{
    connect(loadDefaultButton, &QPushButton::clicked, this, &PointCloudMarkParamterWindow::loadDefaultValue);
    connect(saveButton, &QPushButton::clicked, this, &PointCloudMarkParamterWindow::slotOk);
    connect(cancelButton, &QPushButton::clicked, this, &PointCloudMarkParamterWindow::reject);
}

void PointCloudMarkParamterWindow::initFileType()
{
    formatBox->addItem("PCD", PointCloudFileType::PCD_FILE);
    formatBox->addItem("BIN", PointCloudFileType::BIN_FILE);

    formatBox->setCurrentIndex(static_cast<int>(paramterConfig.getFileType()));
}

