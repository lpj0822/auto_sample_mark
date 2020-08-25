#ifndef SEGMENTATIONLABELCONVERTTHREAD_H
#define SEGMENTATIONLABELCONVERTTHREAD_H

#include <QThread>
#include "helpers/dirprocess.h"
#include "saveMarkData/segmentationmaskprocess.h"
#include "saveMarkData/jsonprocess.h"

class SegmentationLabelConvertThread : public QThread
{
    Q_OBJECT
public:
    SegmentationLabelConvertThread();
    ~SegmentationLabelConvertThread();

    int initData(const QString &fileNameDir);

    void startThread();
    void stopThread();

signals:
    void signalFinish(QString name);

public slots:

protected:
    void run();

private:
    DirProcess dirProcess;
    SegmentationMaskProcess maskProcess;
    JSONProcess jsonProcess;

    QString dirPath;
    bool isStart;

     bool myMakeDir(const QString& pathDir);
    void init();
};

#endif // SEGMENTATIONLABELCONVERTTHREAD_H
