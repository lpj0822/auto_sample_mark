#ifndef POINTCLOUDMARKPARAMTERWINDOW_H
#define POINTCLOUMARKPARAMTERWINDOW_H

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
#include "sampleMarkParam/pointcloudparamterconfig.h"

class PointCloudMarkParamterWindow : public QDialog
{
    Q_OBJECT
public:
    explicit PointCloudMarkParamterWindow(QDialog *parent = 0);
    ~PointCloudMarkParamterWindow();

signals:

public slots:

    void slotOk();
    void loadDefaultValue();

protected:

    void closeEvent(QCloseEvent *event);

private:
    QLabel *formatLabel;
    QComboBox *formatBox;
    QLabel* fieldsNumberLabel;
    QSpinBox* fieldsNumberBox;
    QLabel* infoLabel;
    QPushButton *loadDefaultButton;
    QPushButton *saveButton;
    QPushButton *cancelButton;

    PointCloudParamterConfig paramterConfig;

private:

    void init();
    void initUI();
    void initConnect();

    void initFileType();
};

#endif // POINTCLOUMARKPARAMTERWINDOW_H
