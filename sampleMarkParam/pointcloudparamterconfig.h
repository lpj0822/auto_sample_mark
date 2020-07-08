#ifndef POINTCLOUDPARAMTERCONFIG_H
#define POINTCLOUDPARAMTERCONFIG_H

#include <QString>

typedef enum PointCloudFileType{
    UNPCTYPE = -1,
    PCD_FILE = 0,
    BIN_FILE = 1
}PointCloudFileType;

class PointCloudParamterConfig
{
public:
    PointCloudParamterConfig();
    ~PointCloudParamterConfig();

    void setFieldsNumber(int number);
    void setFileType(int type);

    static int getFieldsNumber();
    static PointCloudFileType getFileType();

    static int loadConfig();
    static int saveConfig();

private:
    static int FIELDS_NUMBER;
    static PointCloudFileType FILE_TYPE;

private:
    void init();
};

#endif // POINTCLOUDPARAMTERCONFIG_H
