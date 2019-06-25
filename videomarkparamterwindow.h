#ifndef VIDEOMARKPARAMTERWINDOW_H
#define VIDEOMARKPARAMTERWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include "sampleMarkParam/videomarkparamterconfig.h"

class VideoMarkParamterWindow : public QDialog
{
    Q_OBJECT
public:
    explicit VideoMarkParamterWindow(QDialog *parent = 0);
    ~VideoMarkParamterWindow();

signals:

public slots:

    void slotOk();
    void loadDefaultValue();

    void slotIsTracking(bool isCheck);

protected:

    void closeEvent(QCloseEvent *event);

private:
    QLabel* skipFrameLabel;
    QSpinBox* skipFrameBox;
    QCheckBox *isTrackingBox;
    QLabel* trackingMethodLabel;
    QComboBox* trackingMethodBox;
    QLabel* infoLabel;
    QPushButton *loadDefaultButton;
    QPushButton *saveButton;
    QPushButton *cancelButton;

    VideoMarkParamterConfig paramterConfig;

private:

    void init();
    void initUI();
    void initConnect();

    void initTrackingMethod();
};

#endif // VIDEOMARKPARAMTERWINDOW_H
