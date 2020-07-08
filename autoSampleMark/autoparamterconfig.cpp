#include "autoparamterconfig.h"

int AutoParamterConfig::inputDataWidth = 512;
int AutoParamterConfig::inputDataHeight = 512;
QString AutoParamterConfig::caffeNet = "";
QString AutoParamterConfig::caffeModel = "";
QMap<int, QString> AutoParamterConfig::modelLabels = QMap<int, QString>();

AutoParamterConfig::AutoParamterConfig()
{
    init();
}

AutoParamterConfig::~AutoParamterConfig()
{

}

void AutoParamterConfig::setInpuDataWidth(int width)
{
    inputDataWidth = width;
}

void AutoParamterConfig::setInpuDataHeight(int height)
{
    inputDataHeight = height;
}

void AutoParamterConfig::setCaffeNet(QString net)
{
    caffeNet = net;
}

void AutoParamterConfig::setCaffeModel(QString model)
{
    caffeModel = model;
}

void AutoParamterConfig::setModelLabels(QMap<int, QString> labels)
{
    modelLabels.clear();
    modelLabels = labels;
}

int AutoParamterConfig::getInpuDataWidth()
{
    return inputDataWidth;
}

int AutoParamterConfig::getInpuDataHeight()
{
    return inputDataHeight;
}

QString AutoParamterConfig::getCaffeNet()
{
    return caffeNet;
}

QString AutoParamterConfig::getCaffeModel()
{
    return caffeModel;
}

QMap<int, QString> AutoParamterConfig::getModelLabels()
{
    return modelLabels;
}

int AutoParamterConfig::loadConfig()
{
    return 0;
}

int AutoParamterConfig::saveConfig()
{
    return 0;
}

void AutoParamterConfig::init()
{

}
