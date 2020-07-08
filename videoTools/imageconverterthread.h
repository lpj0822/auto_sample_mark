#ifndef IMAGECONVERTERTHREAD_H
#define IMAGECONVERTERTHREAD_H

#include <QThread>
#include "helpers/dirprocess.h"
#include "saveData/myimagewriter.h"

class ImageConverterThread : public QThread
{
    Q_OBJECT

public:
    ImageConverterThread();
    ~ImageConverterThread();

    int initImageData(const QString &fileNameDir, const QString& fileSuffix, const QString& imageFormat);

    void startThread();//开始线程
    void stopThread();//结束线程

signals:
    void signalFinish(QString name);

public slots:

protected:
    void run();

private:

    DirProcess *dirProcess;
    MyImageWriter *imageWriter;

    QString dirName;//保存的文件名
    QString suffix;//图片后缀
    QString imageFormat;//转换成的图片格式

    bool isStart;

    void init();
    bool myMakeDir(const QString& pathDir);
};

#endif // IMAGECONVERTERTHREAD_H
