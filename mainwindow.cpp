#pragma execution_character_set("utf-8")
#include "mainwindow.h"
#include <QApplication>
#include <QDesktopServices>
#include <QMessageBox>
#include <QDebug>
#include "helpers/dirprocess.h"
#include "manualparamterconfigwindow.h"
#include "autoSampleMark/autoparamterconfigwindow.h"
#include "videomarkparamterwindow.h"

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
        dirProcess.getDirAllFileName(this->openDataDir, "*.*", processDataList);
        centerWidget->setMarkDataList(this->openDataDir, processDataList, MarkDataType::IMAGE);
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
        dirProcess.getDirAllFileName(this->openDataDir, "*.*", processDataList);
        centerWidget->setMarkDataList(this->openDataDir, processDataList, MarkDataType::VIDEO);
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

void MainWindow::slotCamera()
{
    if(cameraWindow == NULL)
    {
        cameraWindow = new QCameraWindow();
        connect(cameraWindow, &QCameraWindow::signalCloseCameraWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    cameraWindow->show();
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
    centerWidget->setDrawShape(index);
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
    else if(flag.contains("camera"))
    {
        disconnect(cameraWindow, &QCameraWindow::signalCloseCameraWindow, this, &MainWindow::slotCloseOtherWindow);
        delete cameraWindow;
        cameraWindow = NULL;
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
    if(cameraWindow != NULL)
        cameraWindow->close();
    if(centerWidget != NULL)
        centerWidget->close();
    QMainWindow::closeEvent(event);
}

void MainWindow::initData()
{
    autoSampleMarkWindow = NULL;
    videoToPictureWindow = NULL;
    videoFromPictureWindow = NULL;
    videoCuttingWindow = NULL;
    videoCroppingWindow = NULL;
    cameraWindow = NULL;
    openDataDir = ".";
}

void MainWindow::initAction()
{
    //file
    openImageDirAction = new QAction(tr("打开图片文件夹"), this);
    openImageDirAction->setIcon(QIcon(tr(":/images/images/open.png")));
    openVideoDirAction = new QAction(tr("打开视频文件夹"), this);
    openVideoDirAction->setIcon(QIcon(tr(":/images/images/video.png")));
    exitAction = new QAction(tr("退出系统"), this);
    //setting
    manualParamterAction = new QAction(tr("手动标注参数设置"), this);
    autoParamterAction = new QAction(tr("自动标注参数设置"), this);
    videoMarkParamterAction = new QAction(tr("视频标注参数设置"), this);
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
    cameraAction = new QAction(tr("视频采集"), this);
    cameraAction->setIcon(QIcon(tr(":/images/images/record.png")));
    //about
    aboutAction = new QAction(tr("关于"), this);
    userManualAction = new QAction(tr("系统说明"), this);

    //shapeTool
    QMap<int, QString> shapeDatas = myShape.getAllShape();
    shapeWidget = new QWidget(this);
    shapeLabel = new QLabel(tr("选择标注形状"));
    shapeBox = new QComboBox();
    for(QMap<int, QString>::const_iterator iter = shapeDatas.constBegin();
        iter != shapeDatas.constEnd(); ++iter)
    {
        shapeBox->addItem(iter.value(), iter.key());
    }
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
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);
    //setting
    settingMenu = new QMenu(tr("设置"), this);
    settingMenu->addAction(manualParamterAction);
    settingMenu->addAction(autoParamterAction);
    settingMenu->addAction(videoMarkParamterAction);
    //autoMark
    autoMarkMenu = new QMenu(tr("自动化标注"), this);
    autoMarkMenu->addAction(autoMarkAction);
    //tool
    toolMenu = new QMenu(tr("工具"), this);
    toolMenu->addAction(videoToPictureAction);
    toolMenu->addAction(videoFromPictureAction);
    toolMenu->addAction(videoCuttingAction);
    toolMenu->addAction(videoCroppingAction);
    toolMenu->addAction(cameraAction);
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
    //autoMark
    autoMarkTool = new QToolBar(tr("自动化标注"));
    autoMarkTool->setIconSize(QSize(30, 30));
    autoMarkTool->addAction(autoMarkAction);
    //shapeTool
    shapeTool = new QToolBar(tr("标注形状"));
    shapeTool->addWidget(shapeWidget);

    this->addToolBar(fileTool);
    this->addToolBar(autoMarkTool);
    this->addToolBar(shapeTool);
}

void MainWindow::initUI()
{
   centerWidget = new ControlWindow(this);
   this->setCentralWidget(centerWidget);
   //this->setMaximumSize(700,520);
   this->setMinimumSize(1100, 700);
   this->setWindowTitle(tr("样本标注系统"));

   centerWidget->setDrawShape(this->shapeBox->currentData().toInt());
}

void MainWindow::initConnect()
{
    //file
    connect(openImageDirAction, &QAction::triggered, this, &MainWindow::slotOpenImageDir);
    connect(openVideoDirAction, &QAction::triggered, this, &MainWindow::slotOpenVideoDir);
    connect(exitAction, &QAction::triggered, this, &MainWindow::close);
    //setting
    connect(manualParamterAction, &QAction::triggered, this, &MainWindow::slotManualMarkParamterConfig);
    connect(autoParamterAction, &QAction::triggered, this, &MainWindow::slotAutoMarkParamterConfig);
    connect(videoMarkParamterAction, &QAction::triggered, this, &MainWindow::slotVideoMarkParamterConfig);
    //autoMark
    connect(autoMarkAction, &QAction::triggered, this, &MainWindow::slotAutoSampleMark);
    //tool
    connect(videoToPictureAction, &QAction::triggered, this, &MainWindow::slotVideoToPicture);
    connect(videoFromPictureAction, &QAction::triggered, this, &MainWindow::slotVideoFromPicture);
    connect(videoCuttingAction, &QAction::triggered, this, &MainWindow::slotVideoCutting);
    connect(videoCroppingAction, &QAction::triggered, this, &MainWindow::slotVideoCropping);
    connect(cameraAction, &QAction::triggered, this, &MainWindow::slotCamera);
    //about
    connect(aboutAction, &QAction::triggered, this, &MainWindow::slotAbout);
    connect(userManualAction, &QAction::triggered, this, &MainWindow::slotUserManual);

    connect(shapeBox, &QComboBox::currentTextChanged, this, &MainWindow::slotSelectMarkShape);

    connect(this, &MainWindow::signalManualMarkParamterChanged, centerWidget, &ControlWindow::slotManualMarkParamterChanged);
}
