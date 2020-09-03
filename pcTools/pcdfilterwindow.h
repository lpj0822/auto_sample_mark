#ifndef PCDFILTERWINDOW_H
#define PCDFILTERWINDOW_H

#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QList>
#include <QCloseEvent>
#include "pcdfilterterthread.h"

class PCDFilterWindow : public QWidget
{
    Q_OBJECT

public:
    PCDFilterWindow(QWidget *parent = 0);
    ~PCDFilterWindow();

signals:
    void signalClosePCDFilterWindow(QString flag);

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
    QLabel *filterTypeLabel;
    QComboBox *filterTypeBox;

    QString pathDir;
    PCDFilterThread *pcdFilterThread;

    void init();
    void initUI();
    void initConnect();
};

#endif // PCDCONVERTERWINDOW_H
