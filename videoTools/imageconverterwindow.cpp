#pragma execution_character_set("utf-8")
#include "imageconverterwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

ImageConverterWindow::ImageConverterWindow(QWidget *parent) : QWidget(parent)
{
    init();
    initUI();
    initConnect();
}

ImageConverterWindow::~ImageConverterWindow()
{
    if (imageConverterThread != NULL)
    {
        imageConverterThread->stopThread();
        imageConverterThread->wait();
        delete imageConverterThread;
        imageConverterThread = NULL;
    }
}

void ImageConverterWindow::slotOpen()
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

void ImageConverterWindow::slotStart()
{
    QString suffix = imagePostBox->currentText();
    QString imageFormat = imageFormatBox->currentText();
    if (suffix.mid(1) == imageFormat)
    {
        QMessageBox::information(this, tr("提示"), tr("图片类型一样，不需要转换！"));
        return;
    }
    if(imageConverterThread->initImageData(pathDir, suffix, imageFormat) == 0)
    {
        imageConverterThread->startThread();
        imageConverterThread->start();
        stopButton->setEnabled(true);
        startButton->setEnabled(false);
        openButton->setEnabled(false);
    }
}

void ImageConverterWindow::slotStop()
{
    imageConverterThread->stopThread();
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void ImageConverterWindow::slotFinish(QString name)
{
    qDebug() << "save path:" << name;
    stopButton->setEnabled(false);
    startButton->setEnabled(true);
    openButton->setEnabled(true);
}

void ImageConverterWindow::closeEvent(QCloseEvent *event)
{
    if(imageConverterThread->isRunning())
    {
        QMessageBox::StandardButton button;
        button=QMessageBox::question(this,tr("图片格式转换程序"),QString(tr("图片格式转换程序正在运行，是否退出？")),
                                     QMessageBox::Yes|QMessageBox::No);
        if(button==QMessageBox::No)
        {
            event->ignore();
        }
        else if(button==QMessageBox::Yes)
        {
            imageConverterThread->stopThread();
            imageConverterThread->wait();
            event->accept();
            QWidget::closeEvent(event);
            emit signalCloseImageConverterWindow("imageConverter");
        }
    }
    else
    {
        event->accept();
        QWidget::closeEvent(event);
        emit signalCloseImageConverterWindow("imageConverter");
    }
}

void ImageConverterWindow::init()
{
    pathDir = "";
    imageConverterThread = new ImageConverterThread();
}

void ImageConverterWindow::initUI()
{
    mainLayout = new QVBoxLayout();

    QHBoxLayout* layout = new QHBoxLayout();

    imagePostLabel = new QLabel(tr("图片初始格式:"));
    imagePostBox = new QComboBox();
    imagePostBox->addItem(tr("*.png"));
    imagePostBox->addItem(tr("*.jpg"));
    imagePostBox->addItem(tr("*.gif"));
    imagePostBox->addItem(tr("*.pgm"));

    imageFormatLabel = new QLabel(tr("图片转换格式："));
    imageFormatBox = new QComboBox();
    imageFormatBox->addItem(tr(".png"));
    imageFormatBox->addItem(tr(".jpg"));
    imageFormatBox->addItem(tr(".gif"));
    imageFormatBox->addItem(tr(".pgm"));

    layout->addWidget(imagePostLabel);
    layout->addWidget(imagePostBox);
    layout->addWidget(imageFormatLabel);
    layout->addWidget(imageFormatBox);

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
    this->setWindowTitle(tr("图片格式转换"));
}

void ImageConverterWindow::initConnect()
{
    connect(openButton, &QPushButton::clicked, this, &ImageConverterWindow::slotOpen);
    connect(startButton, &QPushButton::clicked, this, &ImageConverterWindow::slotStart);
    connect(stopButton, &QPushButton::clicked, this, &ImageConverterWindow::slotStop);
    connect(imageConverterThread, &ImageConverterThread::signalFinish, this, &ImageConverterWindow::slotFinish);
}
