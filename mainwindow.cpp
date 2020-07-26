#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "mainwindow.h"
#include <QApplication>
#include <QDesktopServices>
#include <QMessageBox>
#include <QDebug>
#include "helpers/dirprocess.h"
#include "paramWindow/manualparamterconfigwindow.h"
#include "paramWindow/videomarkparamterwindow.h"
#include "paramWindow/segmentparamterconfigwindow.h"
#include "autoSampleMark/autoparamterconfigwindow.h"
#include "paramWindow/pointcloudmarkparamterwindow.h"
#include "sampleMarkParam/pointcloudparamterconfig.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    initData();
    initMenuBar();
    initToolBar();
    initUI();
    initConnect();
}

MainWindow::~MainWindow()
{
    centerWidget->deleteLater();
}

void MainWindow::slotOpenImageDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initImageMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        loadDataType = MarkDataType::IMAGE;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenVideoDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initImageMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        loadDataType = MarkDataType::VIDEO;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenImageSegmentDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initSegmentMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        loadDataType = MarkDataType::SEGMENT;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenPCDDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initPointCloudMarkShape();
        if(PointCloudParamterConfig::getFileType() == PointCloudFileType::PCD_FILE)
        {
            dirProcess.getDirAllFileName(this->openDataDir, "*.pcd", processDataList);
        }
        else if(PointCloudParamterConfig::getFileType() == PointCloudFileType::BIN_FILE)
        {
            dirProcess.getDirAllFileName(this->openDataDir, "*.bin", processDataList);
        }
        markWindow[loadDataType]->saveMarkDataList();
        loadDataType = MarkDataType::PCD;
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotManualMarkParamterConfig()
{
    int result = 0;
    ManualParamterConfigWindow *manualMarkParamterWindow = new ManualParamterConfigWindow();
    manualMarkParamterWindow->setModal(true);
    result = manualMarkParamterWindow->exec();
    if(result == QDialog::Accepted)
    {
        emit signalManualMarkParamterChanged();
    }
    delete manualMarkParamterWindow;
    manualMarkParamterWindow = NULL;
}

void MainWindow::slotAutoMarkParamterConfig()
{
    int result = 0;
    AutoParamterConfigWindow *window = new AutoParamterConfigWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    delete window;
    window = NULL;
}

void MainWindow::slotVideoMarkParamterConfig()
{
    int result = 0;
    VideoMarkParamterWindow *window = new VideoMarkParamterWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    delete window;
    window = NULL;
}

void MainWindow::slotSegmentMarkParamterConfig()
{
    int result = 0;
    SegmentParamterConfigWindow *window = new SegmentParamterConfigWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    delete window;
    window = NULL;
}

void MainWindow::slotPointCloudParamterConfig()
{
    int result = 0;
    PointCloudMarkParamterWindow *window = new PointCloudMarkParamterWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    delete window;
    window = NULL;
}

void MainWindow::slotAutoSampleMark()
{
    if(autoSampleMarkWindow == NULL)
    {
        autoSampleMarkWindow = new AutoSampleMarkWindow();
        connect(autoSampleMarkWindow, &AutoSampleMarkWindow::signalCloseAutoSampleMarkWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    autoSampleMarkWindow->show();
    autoSampleMarkWindow->initData();
}

void MainWindow::slotVideoToPicture()
{
    if(videoToPictureWindow == NULL)
    {
        videoToPictureWindow = new FromVideoToPictureWindow();
        connect(videoToPictureWindow, &FromVideoToPictureWindow::signalCloseVideoToPictureWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoToPictureWindow->show();
}

void MainWindow::slotVideoFromPicture()
{
    if(videoFromPictureWindow == NULL)
    {
        videoFromPictureWindow = new FromPictureToVideoWindow();
        connect(videoFromPictureWindow, &FromPictureToVideoWindow::signalCloseVideoFromPictureWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoFromPictureWindow->show();
}

void MainWindow::slotVideoCropping()
{
    if(videoCroppingWindow == NULL)
    {
        videoCroppingWindow = new VideoCroppingWindow();
        connect(videoCroppingWindow, &VideoCroppingWindow::signalCloseVideoCroppingWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoCroppingWindow->show();
}

void MainWindow::slotVideoCutting()
{
    if(videoCuttingWindow == NULL)
    {
        videoCuttingWindow = new VideoCuttingWindow();
        connect(videoCuttingWindow, &VideoCuttingWindow::signalCloseVideoCuttingWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoCuttingWindow->show();
}

void MainWindow::slotImageConverter()
{
    if(imageConverterWindow == NULL)
    {
        imageConverterWindow = new ImageConverterWindow();
        connect(imageConverterWindow, &ImageConverterWindow::signalCloseImageConverterWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    imageConverterWindow->show();
}

void MainWindow::slotCamera()
{
    if(cameraWindow == NULL)
    {
        cameraWindow = new QCameraWindow();
        connect(cameraWindow, &QCameraWindow::signalCloseCameraWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    cameraWindow->show();
}

void MainWindow::slotPcdConverter()
{
    if(pcdConverterWindow == NULL)
    {
        pcdConverterWindow = new PCDConverterWindow();
        connect(pcdConverterWindow, &PCDConverterWindow::signalClosePCDConverterWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    pcdConverterWindow->show();
}

void MainWindow::slotPcdFilter()
{
    if(pcdFilterWindow == NULL)
    {
        pcdFilterWindow = new PCDFilterWindow();
        connect(pcdFilterWindow, &PCDFilterWindow::signalClosePCDFilterWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    pcdFilterWindow->show();
}

void MainWindow::slotAbout()
{
    QMessageBox::about(this, "样本标注系统", "样本标注系统 版本："+ qApp->applicationVersion());
}

void MainWindow::slotUserManual()
{
    QDesktopServices::openUrl(QUrl(tr(":/document/document/user_manuals.pdf"), QUrl::TolerantMode));
}

void MainWindow::slotSelectMarkShape(const QString &text)
{
    int index = this->shapeBox->currentData().toInt();
    switch (loadDataType)
    {
    case MarkDataType::IMAGE:
        markWindow[MarkDataType::IMAGE]->setDrawShape(index);
        break;
    case MarkDataType::VIDEO:
        markWindow[MarkDataType::VIDEO]->setDrawShape(index);
        break;
    case MarkDataType::SEGMENT:
        markWindow[MarkDataType::SEGMENT]->setDrawShape(index);
        break;
    case MarkDataType::PCD:
        break;
    case MarkDataType::UNKNOWN:
        break;
    case MarkDataType::MAX_CONUT:
        break;
    }
}

void MainWindow::slotCloseOtherWindow(QString flag)
{
    if(flag.contains("mark"))
    {
        disconnect(autoSampleMarkWindow, &AutoSampleMarkWindow::signalCloseAutoSampleMarkWindow, this, &MainWindow::slotCloseOtherWindow);
        delete autoSampleMarkWindow;
        autoSampleMarkWindow = NULL;
    }
    else if(flag.contains("videoToPicture"))
    {
        disconnect(videoToPictureWindow, &FromVideoToPictureWindow::signalCloseVideoToPictureWindow, this, &MainWindow::slotCloseOtherWindow);
        delete videoToPictureWindow;
        videoToPictureWindow = NULL;
    }
    else if(flag.contains("videoFromPicture"))
    {
        disconnect(videoFromPictureWindow, &FromPictureToVideoWindow::signalCloseVideoFromPictureWindow, this, &MainWindow::slotCloseOtherWindow);
        delete videoFromPictureWindow;
        videoFromPictureWindow = NULL;
    }
    else if(flag.contains("videoCropping"))
    {
        disconnect(videoCroppingWindow, &VideoCroppingWindow::signalCloseVideoCroppingWindow, this, &MainWindow::slotCloseOtherWindow);
        delete videoCroppingWindow;
        videoCroppingWindow = NULL;
    }
    else if(flag.contains("videoCutting"))
    {
        disconnect(videoCuttingWindow, &VideoCuttingWindow::signalCloseVideoCuttingWindow, this, &MainWindow::slotCloseOtherWindow);
        delete videoCuttingWindow;
        videoCuttingWindow = NULL;
    }
    else if(flag.contains("imageConverter"))
    {
        disconnect(imageConverterWindow, &ImageConverterWindow::signalCloseImageConverterWindow, this, &MainWindow::slotCloseOtherWindow);
        delete imageConverterWindow;
        imageConverterWindow = NULL;
    }
    else if(flag.contains("camera"))
    {
        disconnect(cameraWindow, &QCameraWindow::signalCloseCameraWindow, this, &MainWindow::slotCloseOtherWindow);
        delete cameraWindow;
        cameraWindow = NULL;
    }
    else if(flag.contains("pcdConverter"))
    {
        disconnect(pcdConverterWindow, &PCDConverterWindow::signalClosePCDConverterWindow, this, &MainWindow::slotCloseOtherWindow);
        delete pcdConverterWindow;
        pcdConverterWindow = NULL;
    }
    else if(flag.contains("pcdFilter"))
    {
        disconnect(pcdFilterWindow, &PCDFilterWindow::signalClosePCDFilterWindow, this, &MainWindow::slotCloseOtherWindow);
        delete pcdFilterWindow;
        pcdFilterWindow = NULL;
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if(autoSampleMarkWindow != NULL)
        autoSampleMarkWindow->close();
    if(videoToPictureWindow != NULL)
        videoToPictureWindow->close();
    if(videoFromPictureWindow != NULL)
        videoFromPictureWindow->close();
    if(videoCroppingWindow != NULL)
        videoCroppingWindow->close();
    if(videoCuttingWindow != NULL)
        videoCuttingWindow->close();
    if(imageConverterWindow != NULL)
        imageConverterWindow->close();
    if(cameraWindow != NULL)
        cameraWindow->close();
    if(pcdConverterWindow != NULL)
        pcdConverterWindow->close();
    if(pcdFilterWindow != NULL)
        pcdFilterWindow->close();
    if(centerWidget != NULL)
    {
        centerWidget->currentWidget()->close();
        centerWidget->close();
    }
    QMainWindow::closeEvent(event);
}

void MainWindow::initData()
{
    autoSampleMarkWindow = NULL;
    videoToPictureWindow = NULL;
    videoFromPictureWindow = NULL;
    videoCuttingWindow = NULL;
    videoCroppingWindow = NULL;
    imageConverterWindow = NULL;
    cameraWindow = NULL;

    pcdConverterWindow = NULL;
    pcdFilterWindow = NULL;

    openDataDir = ".";
    loadDataType = MarkDataType::UNKNOWN;
}

void MainWindow::initAction()
{
    //file
    openImageDirAction = new QAction(tr("打开图片文件夹"), this);
    openImageDirAction->setIcon(QIcon(tr(":/images/images/open.png")));
    openVideoDirAction = new QAction(tr("打开视频文件夹"), this);
    openVideoDirAction->setIcon(QIcon(tr(":/images/images/video.png")));
    openSegmentImageDirAction = new QAction(tr("打开图片分割文件夹"), this);
    openSegmentImageDirAction->setIcon(QIcon(tr(":/images/images/open.png")));
    openPCDDirAction = new QAction(tr("打开PCD文件夹"), this);
    openPCDDirAction->setIcon(QIcon(tr(":/images/images/pcl.png")));
    exitAction = new QAction(tr("退出系统"), this);
    //setting
    manualParamterAction = new QAction(tr("手动标注参数设置"), this);
    segmentParamterAction = new QAction(tr("分割标注参数设置"), this);
    autoParamterAction = new QAction(tr("自动标注参数设置"), this);
    videoMarkParamterAction = new QAction(tr("视频标注参数设置"), this);
    pointcloudParamterAction = new QAction(tr("点云标注参数设置"));
    //autoMark
    autoMarkAction = new QAction(tr("自动化样本标注"), this);
    autoMarkAction->setIcon(QIcon(tr(":/images/images/mark.png")));
    //tool
    videoToPictureAction = new QAction(tr("视频转换为图片"), this);
    videoToPictureAction->setIcon(QIcon(tr(":/images/images/cut.png")));
    videoFromPictureAction = new QAction(tr("图片转换为视频"), this);
    videoFromPictureAction->setIcon(QIcon(tr(":/images/images/imageTovideo.png")));
    videoCuttingAction = new QAction(tr("视频剪切"), this);
    videoCuttingAction->setIcon(QIcon(tr(":/images/images/videoCut.png")));
    videoCroppingAction = new QAction(tr("视频剪辑"), this);
    videoCroppingAction->setIcon(QIcon(tr(":/images/images/crop.png")));
    imageConverterAction = new QAction(tr("图片格式转换"), this);
    imageConverterAction->setIcon(QIcon(tr(":/images/images/play.png")));
    cameraAction = new QAction(tr("视频采集"), this);
    cameraAction->setIcon(QIcon(tr(":/images/images/record.png")));

    pcdConverterAction = new QAction(tr("PCD格式转换"), this);
    pcdConverterAction->setIcon(QIcon(tr(":/images/images/pcl.png")));

    pcdFilterAction = new QAction(tr("PCD文件过滤"), this);
    pcdFilterAction->setIcon(QIcon(tr(":/images/images/pcl.png")));

    //about
    aboutAction = new QAction(tr("关于"), this);
    userManualAction = new QAction(tr("系统说明"), this);

    //shapeTool

    shapeWidget = new QWidget(this);
    shapeLabel = new QLabel(tr("选择标注形状"));
    shapeBox = new QComboBox();
    shapeBox->setEnabled(false);
    shapeBox->clear();
    QHBoxLayout *shapLayout = new QHBoxLayout();
    shapLayout->setSpacing(10);
    shapLayout->addWidget(shapeLabel);
    shapLayout->addWidget(shapeBox);
    shapeWidget->setLayout(shapLayout);

}

void MainWindow::initMenuBar()
{
    initAction();
    //file
    fileMenu = new QMenu(tr("文件"), this);
    fileMenu->addAction(openImageDirAction);
    fileMenu->addAction(openVideoDirAction);
    fileMenu->addAction(openSegmentImageDirAction);
    //fileMenu->addAction(openPCDDirAction);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);
    //setting
    settingMenu = new QMenu(tr("设置"), this);
    settingMenu->addAction(manualParamterAction);
    settingMenu->addAction(segmentParamterAction);
    settingMenu->addAction(videoMarkParamterAction);
    settingMenu->addSeparator();
    //settingMenu->addAction(autoParamterAction);
    //settingMenu->addSeparator();
    //settingMenu->addAction(pointcloudParamterAction);
    //autoMark
    autoMarkMenu = new QMenu(tr("自动化标注"), this);
    //autoMarkMenu->addAction(autoMarkAction);
    //tool
    toolMenu = new QMenu(tr("工具"), this);
    toolMenu->addAction(videoToPictureAction);
    toolMenu->addAction(videoFromPictureAction);
    toolMenu->addAction(videoCuttingAction);
    toolMenu->addAction(videoCroppingAction);
    toolMenu->addAction(imageConverterAction);
    toolMenu->addAction(cameraAction);
    toolMenu->addSeparator();
    toolMenu->addAction(pcdConverterAction);
    toolMenu->addAction(pcdFilterAction);
    //about
    aboutMenu = new QMenu(tr("关于"), this);
    aboutMenu->addAction(aboutAction);
    aboutMenu->addAction(userManualAction);

    this->menuBar()->addMenu(fileMenu);
    this->menuBar()->addMenu(settingMenu);
    this->menuBar()->addMenu(autoMarkMenu);
    this->menuBar()->addMenu(toolMenu);
    this->menuBar()->addMenu(aboutMenu);

    this->statusBar()->hide();
}

void MainWindow::initToolBar()
{
    //file
    fileTool = new QToolBar(tr("文件"));
    fileTool->setIconSize(QSize(30, 30));
    fileTool->addAction(openImageDirAction);
    fileTool->addAction(openVideoDirAction);
    fileTool->addAction(openSegmentImageDirAction);
    //fileTool->addAction(openPCDDirAction);
    //autoMark
    autoMarkTool = new QToolBar(tr("自动化标注"));
    autoMarkTool->setIconSize(QSize(30, 30));
    //autoMarkTool->addAction(autoMarkAction);
    //shapeTool
    shapeTool = new QToolBar(tr("标注形状"));
    shapeTool->addWidget(shapeWidget);

    this->addToolBar(fileTool);
    this->addToolBar(autoMarkTool);
    this->addToolBar(shapeTool);
}

void MainWindow::initUI()
{
   centerWidget = new QStackedWidget(this);

   markWindow.clear();
   markWindow.append(new ControlWindow(this));
   markWindow.append(new ImageControlWindow(this));
   markWindow.append(new VideoControlWindow(this));
   markWindow.append(new ImageSegmentControlWindow(this));
   markWindow.append(new PCLControlWindow(this));

   for(int loop = 0; loop < markWindow.size(); loop++)
   {
       centerWidget->addWidget(markWindow[loop]);
   }
   centerWidget->setCurrentIndex(loadDataType);

   this->setCentralWidget(centerWidget);
   //this->setMaximumSize(700,520);
   this->setMinimumSize(1100, 700);
   this->setWindowTitle(tr("样本标注系统"));
}

void MainWindow::initConnect()
{
    //file
    connect(openImageDirAction, &QAction::triggered, this, &MainWindow::slotOpenImageDir);
    connect(openVideoDirAction, &QAction::triggered, this, &MainWindow::slotOpenVideoDir);
    connect(openSegmentImageDirAction, &QAction::triggered, this, &MainWindow::slotOpenImageSegmentDir);
    //connect(openPCDDirAction, &QAction::triggered, this, &MainWindow::slotOpenPCDDir);
    connect(exitAction, &QAction::triggered, this, &MainWindow::close);
    //setting
    connect(manualParamterAction, &QAction::triggered, this, &MainWindow::slotManualMarkParamterConfig);
    connect(segmentParamterAction, &QAction::triggered, this, &MainWindow::slotSegmentMarkParamterConfig);
    //connect(autoParamterAction, &QAction::triggered, this, &MainWindow::slotAutoMarkParamterConfig);
    connect(videoMarkParamterAction, &QAction::triggered, this, &MainWindow::slotVideoMarkParamterConfig);
    //connect(pointcloudParamterAction, &QAction::triggered, this, &MainWindow::slotPointCloudParamterConfig);
    //autoMark
    //connect(autoMarkAction, &QAction::triggered, this, &MainWindow::slotAutoSampleMark);
    //tool
    connect(videoToPictureAction, &QAction::triggered, this, &MainWindow::slotVideoToPicture);
    connect(videoFromPictureAction, &QAction::triggered, this, &MainWindow::slotVideoFromPicture);
    connect(videoCuttingAction, &QAction::triggered, this, &MainWindow::slotVideoCutting);
    connect(videoCroppingAction, &QAction::triggered, this, &MainWindow::slotVideoCropping);
    connect(imageConverterAction, &QAction::triggered, this, &MainWindow::slotImageConverter);
    connect(cameraAction, &QAction::triggered, this, &MainWindow::slotCamera);

    connect(pcdConverterAction, &QAction::triggered, this, &MainWindow::slotPcdConverter);
    connect(pcdFilterAction, &QAction::triggered, this, &MainWindow::slotPcdFilter);

    //about
    connect(aboutAction, &QAction::triggered, this, &MainWindow::slotAbout);
    connect(userManualAction, &QAction::triggered, this, &MainWindow::slotUserManual);

    connect(shapeBox, &QComboBox::currentTextChanged, this, &MainWindow::slotSelectMarkShape);

    for(int loop = 0; loop < markWindow.size(); loop++)
    {
        connect(this, &MainWindow::signalManualMarkParamterChanged, markWindow[loop], &ControlWindow::slotManualMarkParamterChanged);
    }
}

void MainWindow::initImageMarkShape()
{
    QMap<int, QString> shapeDatas = imgShape.getImageShape();
    shapeBox->clear();
    for(QMap<int, QString>::const_iterator iter = shapeDatas.constBegin();
        iter != shapeDatas.constEnd(); ++iter)
    {
        shapeBox->addItem(iter.value(), iter.key());
    }
}

void MainWindow::initSegmentMarkShape()
{
    QMap<int, QString> shapeDatas = imgShape.getSegmentShape();
    shapeBox->clear();
    for(QMap<int, QString>::const_iterator iter = shapeDatas.constBegin();
        iter != shapeDatas.constEnd(); ++iter)
    {
        shapeBox->addItem(iter.value(), iter.key());
    }
}

void MainWindow::initPointCloudMarkShape()
{
    shapeBox->clear();
}
