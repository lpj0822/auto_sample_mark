#ifndef SEGMENTATIONLABELCONVERTWINDOW_H
#define SEGMENTATIONLABELCONVERTWINDOW_H

#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QList>
#include <QCloseEvent>
#include "videoTools/segmentationlabelconvertthread.h"

class SegmentationLabelConvertWindow : public QWidget
{
    Q_OBJECT
public:
    SegmentationLabelConvertWindow(QWidget *parent = nullptr);
    ~SegmentationLabelConvertWindow();

signals:
    void signalCloseSegLabelConverterWindow(QString flag);

public slots:
    void slotOpen();
    void slotStart();
    void slotStop();
    void slotFinish(QString name);

protected:
    void closeEvent(QCloseEvent *event);

private:

    QVBoxLayout *mainLayout;
    QPushButton *openButton;
    QPushButton *startButton;
    QPushButton *stopButton;
    QLineEdit *pathText;
    QLabel *convertTypeLabel;
    QComboBox *convertTypeBox;

    QString pathDir;
    std::unique_ptr<SegmentationLabelConvertThread> converterThread;

    void init();
    void initUI();
    void initConnect();
};

#endif // SEGMENTATIONLABELCONVERTWINDOW_H
