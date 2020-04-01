#pragma execution_character_set("utf-8")
#include "videomarkparamterwindow.h"
#include <QMessageBox>
#include <limits>

VideoMarkParamterWindow::VideoMarkParamterWindow(QDialog *parent) : QDialog(parent), paramterConfig()
{
    init();
    initUI();
    initConnect();
}

VideoMarkParamterWindow::~VideoMarkParamterWindow()
{

}

void VideoMarkParamterWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存视频标注参数配置信息"), tr("是否保存配置信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        slotOk();
    }
    QDialog::closeEvent(event);
}

void VideoMarkParamterWindow::slotOk()
{
    paramterConfig.setSkipFrameNumber(this->skipFrameBox->value());
    paramterConfig.setIsTracking(isTrackingBox->isChecked());
    paramterConfig.setTrackingMethod(trackingMethodBox->currentData().toInt());
    paramterConfig.saveConfig();
    this->accept();
}

void VideoMarkParamterWindow::loadDefaultValue()
{
    this->skipFrameBox->setValue(1);
    this->isTrackingBox->setChecked(true);
    this->trackingMethodBox->setCurrentIndex(0);
}

void VideoMarkParamterWindow::slotIsTracking(bool isCheck)
{
    this->trackingMethodBox->setEnabled(isCheck);
}

void VideoMarkParamterWindow::init()
{
    paramterConfig.loadConfig();
}

void VideoMarkParamterWindow::initUI()
{
    QHBoxLayout *layout = new QHBoxLayout();
    layout->setSpacing(30);
    skipFrameLabel = new QLabel(tr("间隔帧数："));
    skipFrameBox = new QSpinBox();
    skipFrameBox->setMinimum(1);
    skipFrameBox->setValue(paramterConfig.getSkipFrameNumber());
    skipFrameBox->setSingleStep(10);
    layout->addWidget(skipFrameLabel);
    layout->addWidget(skipFrameBox);

    isTrackingBox = new QCheckBox(tr("是否使用跟踪算法"));
    isTrackingBox->setChecked(true);
    trackingMethodLabel = new QLabel(tr("跟踪算法选择："));
    trackingMethodBox = new QComboBox();
    initTrackingMethod();
    QHBoxLayout *layout1 = new QHBoxLayout();
    layout1->setSpacing(30);
    layout1->addWidget(isTrackingBox);
    layout1->addWidget(trackingMethodLabel);
    layout1->addWidget(trackingMethodBox);

    infoLabel = new QLabel(tr("注意：\n该参数配置是针对每一个视频的，表示间隔多少帧标注一次图像。\n"
                              "对于每一个视频保存一个标注文件，文件中的数据格式为json。\n"
                              "每一个要标注的视频不要太长，最后好在3分钟以内。"));

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
    this->setMaximumSize(420, 200);
    this->setMinimumSize(420, 200);
    this->setWindowTitle(tr("视频标注参数配置"));
}

void VideoMarkParamterWindow::initConnect()
{
    connect(loadDefaultButton, &QPushButton::clicked, this, &VideoMarkParamterWindow::loadDefaultValue);
    connect(saveButton, &QPushButton::clicked, this, &VideoMarkParamterWindow::slotOk);
    connect(cancelButton, &QPushButton::clicked, this, &VideoMarkParamterWindow::reject);
    connect(isTrackingBox, &QCheckBox::clicked, this, &VideoMarkParamterWindow::slotIsTracking);
}

void VideoMarkParamterWindow::initTrackingMethod()
{
    trackingMethodBox->addItem("kalman", TrackingMethod::KALMAN);
    trackingMethodBox->addItem("KCF", TrackingMethod::KCF);
    trackingMethodBox->addItem("TLD", TrackingMethod::TLD);
    trackingMethodBox->addItem("MIL", TrackingMethod::MIL);
    trackingMethodBox->addItem("CSRT", TrackingMethod::CSRT);
    trackingMethodBox->addItem("MOSSE", TrackingMethod::MOSSE);

    isTrackingBox->setChecked(paramterConfig.getIsTracking());
    trackingMethodBox->setCurrentIndex((int)paramterConfig.getTrackingMethod());
}

