#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "pcdconverterwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

PCDConverterWindow::PCDConverterWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initConnect();
}

PCDConverterWindow::~PCDConverterWindow()
{
    if (pcdConverterThread != NULL)
    {
        pcdConverterThread->stopThread();
        pcdConverterThread->wait();
        delete pcdConverterThread;
        pcdConverterThread = NULL;
    }
}

void PCDConverterWindow::slotOpen()
{
    pathDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"),tr("."),QFileDialog::ShowDirsOnly);
    if(pathDir.trimmed().isEmpty())
    {
        qDebug() << "打开的文件路径有误:" << pathDir << endl;
        return;
    }
    this->pathText->setText(pathDir);
    startButton->setEnabled(true);
}

void PCDConverterWindow::slotStart()
{
    QString suffix = "*.pcd";
    QString format = formatBox->currentText();
    int fieldsNumber = fieldsNumberBox->value();
    if(pcdConverterThread->initData(pathDir, suffix, format, fieldsNumber) == 0)
    {
        pcdConverterThread->startThread();
        pcdConverterThread->start();
        stopButton->setEnabled(true);
        startButton->setEnabled(false);
        openButton->setEnabled(false);
    }
}

void PCDConverterWindow::slotStop()
{
    pcdConverterThread->stopThread();
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void PCDConverterWindow::slotFinish(QString name)
{
    qDebug() << "save path:" << name;
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void PCDConverterWindow::closeEvent(QCloseEvent *event)
{
    if(pcdConverterThread->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("PCD格式转换程序"),QString(tr("PCD格式转换程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            pcdConverterThread->stopThread();
            pcdConverterThread->wait();
            event->accept();
            QWidget::closeEvent(event);
            emit signalClosePCDConverterWindow("pcdConverter");
        }
    }
    else
    {
        event->accept();
        QWidget::closeEvent(event);
        emit signalClosePCDConverterWindow("pcdConverter");
    }
}

void PCDConverterWindow::init()
{
    pathDir = "";
    pcdConverterThread = new PCDConverterThread();
}

void PCDConverterWindow::initUI()
{
    mainLayout = new QVBoxLayout();

    QHBoxLayout* layout = new QHBoxLayout();

    formatLabel = new QLabel(tr("pcd转换格式："));
    formatBox = new QComboBox();
    formatBox->addItem(tr(".bin"));

    fieldsNumberLabel = new QLabel(tr("转换字段数:"));
    fieldsNumberBox = new QSpinBox();
    fieldsNumberBox->setMinimum(3);
    fieldsNumberBox->setMaximum(4);
    fieldsNumberBox->setSingleStep(1);
    fieldsNumberBox->setValue(3);

    layout->addWidget(formatLabel);
    layout->addWidget(formatBox);
    layout->addWidget(fieldsNumberLabel);
    layout->addWidget(fieldsNumberBox);

    QHBoxLayout* layout1 = new QHBoxLayout();
    openButton = new QPushButton(tr("打开pcd文件夹"));
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
    this->setWindowTitle(tr("PCD格式转换"));
}

void PCDConverterWindow::initConnect()
{
    connect(openButton, &QPushButton::clicked, this, &PCDConverterWindow::slotOpen);
    connect(startButton, &QPushButton::clicked, this, &PCDConverterWindow::slotStart);
    connect(stopButton, &QPushButton::clicked, this, &PCDConverterWindow::slotStop);
    connect(pcdConverterThread, &PCDConverterThread::signalFinish, this, &PCDConverterWindow::slotFinish);
}
