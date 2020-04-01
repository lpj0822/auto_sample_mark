#ifndef SEGMENTPARAMTERCONFIGWINDOW_H
#define SEGMENTPARAMTERCONFIGWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include "utilityGUI/customWindow/markclasstablewidget.h"
#include "sampleMarkParam/segmentparamterconfig.h"

class SegmentParamterConfigWindow : public QDialog
{
    Q_OBJECT
public:
    explicit SegmentParamterConfigWindow(QDialog *parent = nullptr);
    ~SegmentParamterConfigWindow();

signals:

public slots:

    void slotOk();

protected:

    void closeEvent(QCloseEvent *event);

private:
    MarkClassTableWidget *markClassTable;
    QPushButton *loadDefaultButton;
    QPushButton *saveButton;
    QPushButton *cancelButton;

    SegmentParamterConfig paramterConfig;

private:

    void init();
    void initUI();
    void initConnect();

    void initTable();

    void loadDefaultValue();
};

#endif // SEGMENTPARAMTERCONFIGWINDOW_H
