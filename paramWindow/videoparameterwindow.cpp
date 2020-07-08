#pragma execution_character_set("utf-8")
#include "videoparameterwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>

VideoParameterWindow::VideoParameterWindow(int index, int countSec, QDialog *parent) : QDialog(parent), index(index), countSec(countSec)
{
    initUI();
    initConncet();
    initData();
}

void VideoParameterWindow::slotIsProcessAll(bool isChecked)
{
    videoPosBox->setEnabled(!isChecked);
}

void VideoParameterWindow::slotOk()
{
    accept();
}

void VideoParameterWindow::initUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout();
    if(index == 0)
    {
        isAllSaveBox = new QCheckBox(tr("是否全部转换为图片"));
        isAllSaveBox->setChecked(true);

        QHBoxLayout *layout1 = new QHBoxLayout();
        layout1->setSpacing(10);
        startLabel = new QLabel(tr("视频开始转换时间："));
        startPosBox = new QSpinBox();
        startPosBox->setSuffix("s");
        startPosBox->setMinimum(0);
        startPosBox->setValue(0);

        stopLabel = new QLabel(tr("视频结束转换时间："));
        stopPosBox = new QSpinBox();
        stopPosBox->setSuffix("s");
        stopPosBox->setMinimum(0);
        stopPosBox->setValue(0);

        layout1->addWidget(stopLabel);
        layout1->addWidget(startPosBox);
        layout1->addWidget(startLabel);
        layout1->addWidget(stopPosBox);

        videoPosBox = new QGroupBox();
        videoPosBox->setLayout(layout1);
        videoPosBox->setEnabled(false);

        okButton = new QPushButton(tr("确定"));
        cancelButton = new QPushButton(tr("取消"));

        QHBoxLayout *layout2 = new QHBoxLayout();
        layout2->setSpacing(20);
        layout2->setAlignment(Qt::AlignRight);
        layout2->addWidget(okButton);
        layout2->addWidget(cancelButton);

        mainLayout->setSpacing(10);
        mainLayout->addWidget(isAllSaveBox);
        mainLayout->addWidget(videoPosBox);
        mainLayout->addLayout(layout2);
    }
    else if(index == 1)
    {
        QHBoxLayout *layout = new QHBoxLayout();
        layout->setSpacing(30);
        skipFrameLabel = new QLabel(tr("间隔帧数："));
        skipFrameBox = new QSpinBox();
        skipFrameBox->setMinimum(1);
        skipFrameBox->setValue(10);
        skipFrameBox->setSingleStep(10);
        layout->addWidget(skipFrameLabel);
        layout->addWidget(skipFrameBox);
        infoLabel = new QLabel(tr("注意：\n该参数配置是针对每一个视频的，表示间隔多少帧标注一次图像。\n"
                                  "对于每一个视频保存一个标注文件，文件中的数据格式为json。\n"
                                  "每一个要标注的视频不要太长，最后好在3分钟以内。"));
        mainLayout->setSpacing(20);
        mainLayout->addLayout(layout);
        mainLayout->addWidget(infoLabel);
    }

    this->setLayout(mainLayout);
    this->setMaximumSize(420, 160);
    this->setMinimumSize(420, 160);
    this->setWindowTitle(tr("视频参数配置"));
}

void VideoParameterWindow::initConncet()
{
    if(index == 0)
    {
        connect(isAllSaveBox, &QCheckBox::clicked, this, &VideoParameterWindow::slotIsProcessAll);
        connect(okButton, &QPushButton::clicked, this, &VideoParameterWindow::slotOk);
        connect(cancelButton, &QPushButton::clicked, this, &VideoParameterWindow::reject);
    }
}

void VideoParameterWindow::initData()
{
    if(index == 0)
    {
        startPosBox->setMaximum(countSec);
        stopPosBox->setMaximum(countSec);
    }
}
