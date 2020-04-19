#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "imagesegmentcontrolwindow.h"
#include <QMessageBox>
#include <QDebug>
#include "sampleMarkParam/segmentparamterconfig.h"

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

    updateIsMarkButton(this->isMark);
    updateDrawLabel(this->isMark);

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
    this->drawLable->setDrawShape(shapeId);
}

void ImageSegmentControlWindow::slotIsMark()
{
    if(currentIndex >= 0)
    {
        if(isMark)
        {
            this->isMark = false;
        }
        else
        {
            this->isMark = true;
        }
        updateIsMarkButton(this->isMark);
        updateDrawLabel(this->isMark);
    }
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

void ImageSegmentControlWindow::updateDrawLabel(bool isValue)
{
    this->drawLable->setEnabled(isValue);
}

void ImageSegmentControlWindow::updateImage()
{
    drawLable->clearObjects();
    drawLable->setNewQImage(currentImage);
}

void ImageSegmentControlWindow::loadMarkData(const QString dataPath)
{   QString saveSegmentLabelDir = this->markDataDir + "/../" + "Segmentation";
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::SEGMENT)
    {
        loadImageData(dataPath, saveSegmentLabelDir, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageSegmentControlWindow::saveMarkDataResult()
{
    QDir makeDir;
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    QString saveSegmentationDir = this->markDataDir + "/../" + "Segmentation";
    QList<MyObject> objects = drawLable->getObjects();
    MyObject maskImage = drawLable->getSegmentMask();
    if(objects.size() > 0)
    {
        if(!makeDir.exists(saveAnnotationsDir))
        {
            if(!makeDir.mkdir(saveAnnotationsDir))
            {
                qDebug() << "make Annotations dir fail!" << endl;
            }
        }
        if(!makeDir.exists(saveSegmentationDir))
        {
            if(!makeDir.mkdir(saveSegmentationDir))
            {
                qDebug() << "make Segmentation dir fail!" << endl;
            }
        }
        if(this->markDataType == MarkDataType::SEGMENT)
        {
            saveImageDataResult(saveAnnotationsDir, this->currentImagePath, objects);
            saveImageSegmentResult(saveSegmentationDir, this->currentImagePath, maskImage);
        }
    }
}

void ImageSegmentControlWindow::loadMarkImage()
{
    QString saveSegmentLabelDir = this->markDataDir + "/../" + "Segmentation";
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    if(this->markDataType == MarkDataType::SEGMENT && processMarkDataList.size() > 0)
    {
        currentImagePath =  processMarkDataList[currentIndex];
        loadImageData(currentImagePath, saveSegmentLabelDir, saveAnnotationsDir);
    }
    updateListBox();
    updateMarkProcessLable();
}

void ImageSegmentControlWindow::saveMarkImageResult()
{
    QDir makeDir;
    QString saveAnnotationsDir = this->markDataDir + "/../" + "Annotations";
    QList<MyObject> objects = drawLable->getObjects();
    if(objects.size() > 0)
    {
        if(!makeDir.exists(saveAnnotationsDir))
        {
            if(!makeDir.mkdir(saveAnnotationsDir))
            {
                qDebug() << "make Annotations dir fail!" << endl;
            }
        }
        if(this->markDataType == MarkDataType::SEGMENT)
        {
            saveImageDataResult(saveAnnotationsDir, this->currentImagePath, objects);
        }
    }
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
    SegmentParamterConfig::loadConfig();
}

void ImageSegmentControlWindow::initConnect()
{
    connect(markDataListWidget, &QListWidget::itemClicked, this, &ImageSegmentControlWindow::slotImageItem);
    connect(drawMarkDataWidget, &MyStackedWidget::signalsKey, this, &ImageSegmentControlWindow::slotScrollArea);
    connect(isMarkButton, &QPushButton::clicked, this, &ImageSegmentControlWindow::slotIsMark);
    connect(classBox, &QComboBox::currentTextChanged, this, &ImageSegmentControlWindow::slotChangeClass);
}

void ImageSegmentControlWindow::initImageList()
{
    processDataFlagList.clear();
    readMarkHistory();
}

void ImageSegmentControlWindow::loadImageData(const QString &imagePath, const QString &saveSegmentLabelDir,
                                              const QString &saveAnnotationsDir)
{
    if(currentImage.load(imagePath))
    {
        currentImagePath = imagePath;
        updateImage();
        QFileInfo imageFileInfo(currentImagePath);
        QString readSegmentPath = saveSegmentLabelDir + "/" + imageFileInfo.completeBaseName() + ".png";
        QFileInfo segmentFileInfo(readSegmentPath);
        MyObject maskObject;
        if(segmentFileInfo.exists() && segmentImageProcess.readSegmentImage(readSegmentPath, maskObject) == 0)
        {

        }
        QString readJsonPath= saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".json";
        QFileInfo jsonFileInfo(readJsonPath);
        QList<MyObject> objects;
        if(jsonFileInfo.exists() && jsonProcess.readJSON(readJsonPath, objects) == 0)
        {

        }
        drawLable->setOjects(maskObject, objects, classBox->currentText());
    }
    else
    {
        QMessageBox::information(this, tr("加载图片"), tr("加载图片失败！"));
    }
}

void ImageSegmentControlWindow::saveImageDataResult(const QString &saveAnnotationsDir, const QString &imagePath,
                                             const QList<MyObject> &objects)
{
    QFileInfo imageFileInfo(imagePath);
    QString saveXmlPath = saveAnnotationsDir + "/" + imageFileInfo.completeBaseName() + ".xml";
    QFileInfo xmlFileInfo(saveXmlPath);

    QMessageBox::StandardButton result = QMessageBox::question(this, tr("保存标注信息"), tr("是否保存标注信息?"),
                                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        if(objects.size() > 0)
        {
            if(xmlProcess.createXML(saveXmlPath, currentImagePath, currentImage.width(),
                                    currentImage.height(), objects) == 0)
            {
                if(currentIndex >= 0)
                {
                    processDataFlagList[currentIndex] = 0;
                }
            }
            else
            {
                QMessageBox::information(this, tr("保存XML"), tr("保存XML文件失败！"));
            }
        }
        else if(xmlFileInfo.exists())
        {
            QFile tempFile(saveXmlPath);
            tempFile.remove();
        }
    }
}

void ImageSegmentControlWindow::saveImageSegmentResult(const QString &saveAnnotationsDir, const QString &imagePath,
                                                const MyObject &maskObject)
{
    QFileInfo imageFileInfo(imagePath);
    QString saveImagePath = saveAnnotationsDir + "/" + imageFileInfo.fileName();
    QFileInfo saveFileInfo(saveImagePath);
    QImage result = maskObject.getSegmentImage();
    if(!result.isNull())
    {
        if(!result.save(saveImagePath))
        {
            QMessageBox::information(this, tr("保存Segment图象"), tr("保存Segment图象%1失败！").arg(saveImagePath));
        }
    }
    else if(saveFileInfo.exists())
    {
        QFile tempFile(saveImagePath);
        tempFile.remove();
    }
}

void ImageSegmentControlWindow::initImageData()
{
    this->currentImagePath = "";
}

