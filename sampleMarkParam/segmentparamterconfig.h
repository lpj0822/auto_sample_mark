#ifndef SEGMENTPARAMTERCONFIG_H
#define SEGMENTPARAMTERCONFIG_H

#include <QString>
#include <QList>
#include <QMap>

class SegmentParamterConfig
{
public:
    SegmentParamterConfig();
    ~SegmentParamterConfig();

    void setLineWidth(const int width);
    static int getLineWidth();

    static int loadClassConfig(const QString &classPath);
    static int saveClassConfig(const QString &saveClassPath);

    static int loadConfig();
    static int saveConfig();

private:

    static int LINE_WIDTH;

private:

    void init();
};

#endif // SEGMENTPARAMTERCONFIG_H
