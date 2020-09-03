#include "pointcloudparamterconfig.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QFile>
#include <QFileInfo>
#include <QVariant>

int PointCloudParamterConfig::FIELDS_NUMBER = 3;
PointCloudFileType PointCloudParamterConfig::FILE_TYPE = PointCloudFileType::PCD_FILE;

PointCloudParamterConfig::PointCloudParamterConfig()
{

}


PointCloudParamterConfig::~PointCloudParamterConfig()
{

}

void PointCloudParamterConfig::setFieldsNumber(int number)
{
    FIELDS_NUMBER = number;
}

void PointCloudParamterConfig::setFileType(int type)
{
    switch (type) {
    case -1:
        FILE_TYPE = PointCloudFileType::UNPCTYPE;
        break;
    case 0:
        FILE_TYPE = PointCloudFileType::PCD_FILE;
        break;
    case 1:
        FILE_TYPE = PointCloudFileType::BIN_FILE;
        break;
    }
}

int PointCloudParamterConfig::getFieldsNumber()
{
    return FIELDS_NUMBER;
}

PointCloudFileType PointCloudParamterConfig::getFileType()
{
    return FILE_TYPE;
}

int PointCloudParamterConfig::loadConfig()
{
    QByteArray data;
    QFile file;
    file.setFileName("./pointcloud_mark_config.json");
    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        return -1;
    }
    data = file.readAll();
    file.close();
    QJsonParseError jsonError;
    QJsonDocument parseDoucment = QJsonDocument::fromJson(QString(data).toUtf8(), &jsonError);
    if(jsonError.error == QJsonParseError::NoError)
    {
        if (!(parseDoucment.isNull() || parseDoucment.isEmpty()))
        {
            if (parseDoucment.isObject())
            {
                QJsonObject jsonObject = parseDoucment.object();
                if(jsonObject.contains("fieldsNumber"))
                {
                    FIELDS_NUMBER = jsonObject.take("fieldsNumber").toVariant().toInt();
                }
                if(jsonObject.contains("fileType"))
                {
                    int type = jsonObject.take("fileType").toVariant().toInt();
                    switch (type) {
                    case -1:
                        FILE_TYPE = PointCloudFileType::UNPCTYPE;
                        break;
                    case 0:
                        FILE_TYPE = PointCloudFileType::PCD_FILE;
                        break;
                    case 1:
                        FILE_TYPE = PointCloudFileType::BIN_FILE;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        return -2;
    }
    return 0;
}

int PointCloudParamterConfig::saveConfig()
{
    QJsonDocument doc;
    QByteArray data;
    QJsonObject jsonData;
    QFile file("./pointcloud_mark_config.json");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate |QIODevice::Text))
    {
        return -1;
    }
    jsonData.insert("fieldsNumber", QString::number(FIELDS_NUMBER));
    jsonData.insert("fileType", QString::number(FILE_TYPE));

    doc.setObject(jsonData);
    data = doc.toJson();
    file.write(data);
    file.close();
    return 0;
}
