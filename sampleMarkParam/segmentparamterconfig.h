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

    void setNearPointLength(int lenght);
    void setMarkClass(QMap<QString, QString> classMark);
    void addMarkClass(QString className, QString classColor);
    void removeMarkClass(QString className);

    static int getNearPointLenght();
    static QMap<QString, QString> getMarkClass();
    static QString getMarkClassColor(QString className);

    static int loadClassConfig(const QString &classPath);
    static int loadConfig();
    static int saveConfig();

private:

    static int NEAR_POINT_LENGTH;

    static QMap<QString, QString> markClass;

private:

    void init();
};

#endif // SEGMENTPARAMTERCONFIG_H
