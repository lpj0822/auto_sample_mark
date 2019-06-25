#ifndef PCLCONTROLWINDOW_H
#define PCLCONTROLWINDOW_H

#include "controlwindow.h"
#include "drawWidget/pclviewer.h"

class PCLControlWindow : public ControlWindow
{
    Q_OBJECT

public:
    PCLControlWindow(QWidget *parent = 0);
    ~PCLControlWindow();

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType);

public slots:

    void slotIsMark();
    void slotImageItem(QListWidgetItem *item);
    void slotScrollArea(int keyValue);

protected:
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent *e);

private:
    void showPrevious();
    void showNext();

    void loadPointCloud();
    void loadPointCloudData(const QString pcdFilePath);

    void initDrawWidget();
    void initData();
    void initConnect();

private:
    PCLViewer *drawPointCloud;

private:

    QString currentPCDPath;
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud;

};

#endif // PCLCONTROLWINDOW_H
