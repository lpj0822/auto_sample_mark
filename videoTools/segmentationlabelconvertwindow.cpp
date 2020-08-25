#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "segmentationlabelconvertwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include "sampleMarkParam/manualparamterconfig.h"

SegmentationLabelConvertWindow::SegmentationLabelConvertWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initConnect();
}

SegmentationLabelConvertWindow::~SegmentationLabelConvertWindow()
{
    if (converterThread.get() != nullptr)
    {
        converterThread->stopThread();
        converterThread->wait();
    }
}

void SegmentationLabelConvertWindow::slotOpen()
{
    pathDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), pathDir, QFileDialog::ShowDirsOnly);
    if(pathDir.trimmed().isEmpty())
    {
        qDebug() << "打开的文件路径有误:" << pathDir << endl;
        return;
    }
    this->pathText->setText(pathDir);
    QString saveClassPath = pathDir + "/../" + "class.json";
    if(ManualParamterConfig::loadSegClassConfig(saveClassPath) == 0)
    {
        startButton->setEnabled(true);
    }
}

void SegmentationLabelConvertWindow::slotStart()
{
    if(converterThread->initData(pathDir) == 0)
    {
        converterThread->startThread();
        converterThread->start();
        stopButton->setEnabled(true);
        startButton->setEnabled(false);
        openButton->setEnabled(false);
    }
}

void SegmentationLabelConvertWindow::slotStop()
{
    converterThread->stopThread();
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void SegmentationLabelConvertWindow::slotFinish(QString name)
{
    qDebug() << "save path:" << name;
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void SegmentationLabelConvertWindow::closeEvent(QCloseEvent *event)
{
    if(converterThread->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("分割图生成器程序"),QString(tr("分割图生成器程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            converterThread->stopThread();
            converterThread->wait();
            event->accept();
            emit signalCloseSegLabelConverterWindow("segLabelConverter");
            QWidget::closeEvent(event);
        }
    }
    else
    {
        event->accept();
        emit signalCloseSegLabelConverterWindow("segLabelConverter");
        QWidget::closeEvent(event);
    }
}

void SegmentationLabelConvertWindow::init()
{
    pathDir = ".";
    converterThread = std::unique_ptr<SegmentationLabelConvertThread>(new SegmentationLabelConvertThread());
}

void SegmentationLabelConvertWindow::initUI()
{
    mainLayout = new QVBoxLayout();

    QHBoxLayout* layout = new QHBoxLayout();

    convertTypeLabel = new QLabel(tr("分割图生成:"));
    convertTypeBox = new QComboBox();
    convertTypeBox->addItem(tr("彩色图"));
    convertTypeBox->addItem(tr("标签图"));

    layout->addWidget(convertTypeLabel);
    layout->addWidget(convertTypeBox);

    QHBoxLayout* layout1 = new QHBoxLayout();
    openButton = new QPushButton(tr("打开图片文件夹"));
    pathText = new QLineEdit();
    pathText->setReadOnly(true);
    layout1->addWidget(pathText);
    layout1->addWidget(openButton);

    QHBoxLayout*layout2 = new QHBoxLayout();
    startButton = new QPushButton(tr("开始转换"));
    stopButton = new QPushButton(tr("结束转换"));
    startButton->setEnabled(false);
    stopButton->setEnabled(false);
    layout2->setSpacing(50);
    layout2->addWidget(startButton);
    layout2->addWidget(stopButton);

    mainLayout->addLayout(layout);
    mainLayout->addLayout(layout1);
    mainLayout->addLayout(layout2);
    this->setLayout(mainLayout);
    this->setMinimumSize(400, 200);
    this->setWindowTitle(tr("分割图生成器"));
}

void SegmentationLabelConvertWindow::initConnect()
{
    connect(openButton, &QPushButton::clicked, this, &SegmentationLabelConvertWindow::slotOpen);
    connect(startButton, &QPushButton::clicked, this, &SegmentationLabelConvertWindow::slotStart);
    connect(stopButton, &QPushButton::clicked, this, &SegmentationLabelConvertWindow::slotStop);
    connect(converterThread.get(), &SegmentationLabelConvertThread::signalFinish, this, &SegmentationLabelConvertWindow::slotFinish);
}
