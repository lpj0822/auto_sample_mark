#ifndef PCLCONTROLWINDOW_H
#define PCLCONTROLWINDOW_H

#include "controlwindow.h"
#include "drawWidget/pclviewer.h"

class PCLControlWindow : public ControlWindow
{
    Q_OBJECT

public:
    PCLControlWindow(QWidget *parent = 0);
    ~PCLControlWindow() override;

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType) override;

public slots:

    void slotIsMark();
    void slotImageItem(QListWidgetItem *item);
    void slotScrollArea(int keyValue);

protected:
    void closeEvent(QCloseEvent *event) override;
    void keyPressEvent(QKeyEvent *e) override;

private:
    void showPrevious();
    void showNext();

    void loadMarkPointCloud();
    void loadPointCloudData(const QString pcdFilePath);

    void initDrawWidget();
    void initData();
    void initConnect();

private:
    PCLViewer *drawPointCloud;

private:

    QString currentPCDPath;
};

#endif // PCLCONTROLWINDOW_H
