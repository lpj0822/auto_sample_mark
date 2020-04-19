#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "pcdfilterwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

PCDFilterWindow::PCDFilterWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initConnect();
}

PCDFilterWindow::~PCDFilterWindow()
{
    if (pcdFilterThread != NULL)
    {
        pcdFilterThread->stopThread();
        pcdFilterThread->wait();
        delete pcdFilterThread;
        pcdFilterThread = NULL;
    }
}

void PCDFilterWindow::slotOpen()
{
    pathDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"),tr("."),QFileDialog::ShowDirsOnly);
    if(pathDir.trimmed().isEmpty())
    {
        // qDebug() << "file dir error:" << pathDir << endl;
        return;
    }
    this->pathText->setText(pathDir);
    startButton->setEnabled(true);
}

void PCDFilterWindow::slotStart()
{
    QString suffix = "*.pcd";
    int flag = filterTypeBox->currentData().toInt();
    if(pcdFilterThread->initData(pathDir, suffix, flag) == 0)
    {
        pcdFilterThread->startThread();
        pcdFilterThread->start();
        stopButton->setEnabled(true);
        startButton->setEnabled(false);
        openButton->setEnabled(false);
    }
}

void PCDFilterWindow::slotStop()
{
    pcdFilterThread->stopThread();
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void PCDFilterWindow::slotFinish(QString name)
{
    qDebug() << "save path:" << name;
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void PCDFilterWindow::closeEvent(QCloseEvent *event)
{
    if(pcdFilterThread->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("PCD文件过滤程序"),QString(tr("PCD文件过滤程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            pcdFilterThread->stopThread();
            pcdFilterThread->wait();
            event->accept();
            QWidget::closeEvent(event);
            emit signalClosePCDFilterWindow("pcdFilter");
        }
    }
    else
    {
        event->accept();
        QWidget::closeEvent(event);
        emit signalClosePCDFilterWindow("pcdFilter");
    }
}

void PCDFilterWindow::init()
{
    pathDir = "";
    pcdFilterThread = new PCDFilterThread();
}

void PCDFilterWindow::initUI()
{
    mainLayout = new QVBoxLayout();

    QHBoxLayout* layout = new QHBoxLayout();

    filterTypeLabel = new QLabel(tr("PCD文件过滤类型:"));
    filterTypeBox = new QComboBox();
    filterTypeBox->addItem(tr("删除损坏文件"), 0);

    layout->addWidget(filterTypeLabel);
    layout->addWidget(filterTypeBox);

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
    this->setWindowTitle(tr("PCD文件过滤"));
}

void PCDFilterWindow::initConnect()
{
    connect(openButton, &QPushButton::clicked, this, &PCDFilterWindow::slotOpen);
    connect(startButton, &QPushButton::clicked, this, &PCDFilterWindow::slotStart);
    connect(stopButton, &QPushButton::clicked, this, &PCDFilterWindow::slotStop);
    connect(pcdFilterThread, &PCDFilterThread::signalFinish, this, &PCDFilterWindow::slotFinish);
}
