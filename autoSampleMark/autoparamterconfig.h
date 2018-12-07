#ifndef AUTOPARAMTERCONFIG_H
#define AUTOPARAMTERCONFIG_H

#include <QString>
#include <QMap>

class AutoParamterConfig
{
public:
    AutoParamterConfig();
    ~AutoParamterConfig();

    void setInpuDataWidth(int width);
    void setInpuDataHeight(int height);
    void setCaffeNet(QString net);
    void setCaffeModel(QString model);
    void setModelLabels(QMap<int, QString> labels);

    static int getInpuDataWidth();
    static int getInpuDataHeight();
    static QString getCaffeNet();
    static QString getCaffeModel();
    static QMap<int, QString> getModelLabels();
    static int loadConfig();
    static int saveConfig();

private:

    static int inputDataWidth;
    static int inputDataHeight;
    static QString caffeNet;
    static QString caffeModel;
    static QMap<int, QString> modelLabels;

private:

    void init();

};

#endif // AUTOPARAMTERCONFIG_H
