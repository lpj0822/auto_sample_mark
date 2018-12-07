#ifndef MANUALPARAMTERCONFIG_H
#define MANUALPARAMTERCONFIG_H

#include <QString>
#include <QList>
#include <QMap>

class ManualParamterConfig
{
public:
    ManualParamterConfig();
    ~ManualParamterConfig();

    void setMinWidth(int minWidth);
    void setMinHeight(int minHeight);
    void setNearPointLength(int lenght);
    void setMinScale(int minScale);
    void setMaxScale(int maxScale);
    void setMarkClass(QMap<QString, QString> classMark);
    void addMarkClass(QString className, QString classColor);
    void removeMarkClass(QString className);

    static int getMinWidth();
    static int getMinHeight();
    static int getNearPointLenght();
    static QMap<QString, QString> getMarkClass();
    static QString getMarkClassColor(QString className);

    static int loadConfig();
    static int saveConfig();

private:

    static int MIN_WIDTH;
    static int MIN_HEIGHT;

    static int NEAR_POINT_LENGTH;

    static int MAX_SCALE;

    static int MIN_SCALE;

    static QMap<QString, QString> markClass;

private:

    void init();
};

#endif // MANUALPARAMTERCONFIG_H
