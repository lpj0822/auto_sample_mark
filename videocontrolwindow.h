#ifndef VIDEOCONTROLWINDOW_H
#define VIDEOCONTROLWINDOW_H

#include "controlwindow.h"

class VideoControlWindow : public ControlWindow
{
public:
    VideoControlWindow();
    ~VideoControlWindow();

    void setMarkDataList(const QString markDataDir, const QList<QString> markDataList, const MarkDataType dataType);
};

#endif // VIDEOCONTROLWINDOW_H
