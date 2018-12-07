#ifndef VIDEOPARAMETERWINDOW_H
#define VIDEOPARAMETERWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QCheckBox>
#include <QGroupBox>

class VideoParameterWindow : public QDialog
{
    Q_OBJECT
public:
    explicit VideoParameterWindow(int index, int countSec =0, QDialog *parent = 0);

    //0
    QCheckBox* isAllSaveBox;
    QGroupBox *videoPosBox;
    QSpinBox* startPosBox;
    QSpinBox* stopPosBox;

    //1
    QSpinBox* skipFrameBox;

signals:

public slots:
    void slotIsProcessAll(bool isChecked);
    void slotOk();

private:
    void initUI();
    void initConncet();
    void initData();

private:
    //0
    QLabel* startLabel;
    QLabel* stopLabel;

    QPushButton *okButton;
    QPushButton *cancelButton;

    //1
    QLabel* skipFrameLabel;
    QLabel* infoLabel;

private:

    int index;

    int countSec;
};

#endif // VIDEOPARAMETERWINDOW_H
