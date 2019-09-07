#pragma execution_character_set("utf-8")
#include "imagesegmentcontrolwindow.h"
#include <QMessageBox>
#include <QDebug>

ImageSegmentControlWindow::ImageSegmentControlWindow(QWidget *parent)
    : ControlWindow(parent)
{
    initDrawWidget();
    initData();
    initConnect();
}

ImageSegmentControlWindow::~ImageSegmentControlWindow()
{

}

void ImageSegmentControlWindow::setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType)
{
    readClassConfig(markDataDir);
    initMarkData(markDataDir, dataType);
    initImageData();

    if(markDataList.size() > 0)
    {
        this->processMarkDataList = markDataList;
        initImageList();
        updateListBox();
        updateMarkProcessLable();
        isMarkButton->setEnabled(true);
    }
    else
    {
        isMarkButton->setEnabled(false);
    }
    this->setFocus();
}

void ImageSegmentControlWindow::saveMarkDataList()
{
    writeMarkHistory();
}

void ImageSegmentControlWindow::setDrawShape(int shapeId)
{
    qDebug() << "shape:" << shapeId;
}

void ImageSegmentControlWindow::slotImageItem(QListWidgetItem *item)
{
    this->currentIndex = markDataListWidget->row(item);
    loadMarkData(item->text());
    this->setFocus();
}

void ImageSegmentControlWindow::slotChangeClass(QString classText)
{
    loadMarkImage();
    this->setFocus();
}

void ImageSegmentControlWindow::slotScrollArea(int keyValue)
{
    if(processMarkDataList.size() > 0)
    {
        if(keyValue == int(Qt::Key_A))
        {
            showPrevious();
        }
        else if(keyValue == int(Qt::Key_D))
        {
            showNext();
        }
    }
    if(keyValue == int(Qt::Key_Escape))
    {
        slotShowFull();
    }
}

void ImageSegmentControlWindow::closeEvent(QCloseEvent *event)
{
    saveMarkDataList();
    ControlWindow::closeEvent(event);
}

void ImageSegmentControlWindow::keyPressEvent(QKeyEvent *e)
{
    if(processMarkDataList.size() > 0)
    {
        if(e->key() == Qt::Key_A)
        {
            showPrevious();
        }
        else if(e->key() == Qt::Key_D)
        {
            showNext();
        }
    }
}

void ImageSegmentControlWindow::showPrevious()
{
    if(this->markDataType == MarkDataType::SEGMENT)
    {
        if(currentIndex > 0)
        {
            currentIndex--;
            loadMarkImage();
        }
    }
}

void ImageSegmentControlWindow::showNext()
{
    if(currentIndex < processMarkDataList.size() - 1)
    {
        currentIndex++;
        loadMarkImage();
    }
}

void ImageSegmentControlWindow::updateImage()
{
    drawLable->clearObjects();
    drawLable->setNewQImage(currentImage);
}

void ImageSegmentControlWindow::loadMarkData(const QString dataPath)
{   QString saveSegmentationDir = this->markDataDir + "/../" + "Segmentation";
    if(this->markDataType == MarkDataType::SEGMENT)
    {
        loadImageData(dataPath, saveSegmentationDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageSegmentControlWindow::loadMarkImage()
{
    QString saveSegmentationDir = this->markDataDir + "/../" + "Segmentation";
    if(this->markDataType == MarkDataType::SEGMENT && processMarkDataList.size() > 0)
    {
        currentImagePath =  processMarkDataList[currentIndex];
        loadImageData(currentImagePath, saveSegmentationDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageSegmentControlWindow::initDrawWidget()
{
    drawLable = new SegmentLabel();
    drawLableScrollArea = new QScrollArea();
    drawLableScrollArea->setAlignment(Qt::AlignCenter);
    drawLableScrollArea->setBackgroundRole(QPalette::Dark);
    drawLableScrollArea->setAutoFillBackground(true);
    //drawScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //drawScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    drawLableScrollArea->setWidget(drawLable);
    drawMarkDataWidget->addWidget(drawLableScrollArea);

    drawLable->clearObjects();
    drawLable->setNewQImage(currentImage);
    drawLable->setEnabled(true);
    drawMarkDataWidget->setCurrentIndex(1);
}

void ImageSegmentControlWindow::initData()
{
    initMarkData(".", MarkDataType::SEGMENT);
    initImageData();
}

void ImageSegmentControlWindow::initConnect()
{
    connect(markDataListWidget, &QListWidget::itemClicked, this, &ImageSegmentControlWindow::slotImageItem);
    connect(drawMarkDataWidget, &MyStackedWidget::signalsKey, this, &ImageSegmentControlWindow::slotScrollArea);
    connect(classBox, &QComboBox::currentTextChanged, this, &ImageSegmentControlWindow::slotChangeClass);
}

void ImageSegmentControlWindow::initImageList()
{
    processDataFlagList.clear();
    readMarkHistory();
}

void ImageSegmentControlWindow::loadImageData(const QString imagePath, const QString saveAnnotationsDir)
{
    if(currentImage.load(imagePath))
    {
        currentImagePath = imagePath;
        updateImage();
        QFileInfo imageFileInfo(currentImagePath);
        QString readSegmentPath = saveAnnotationsDir + "/" + imageFileInfo.fileName();
        QFileInfo segmentFileInfo(readSegmentPath);
        MyObject object;
        if(segmentFileInfo.exists() && segmentImageProcess.readSegmentImage(readSegmentPath, object) == 0)
        {
            drawLable->setOjects(object, classBox->currentText());
        }
    }
    else
    {
        QMessageBox::information(this, tr("加载图片"), tr("加载图片失败！"));
    }
}

void ImageSegmentControlWindow::initImageData()
{
    this->currentImagePath = "";
}

