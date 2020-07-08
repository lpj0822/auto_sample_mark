#include "myvideowriter.h"
#include <iostream>

// Transform from int to char via Bitwise operators
int ex=-1;
char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};

MyVideoWriter::MyVideoWriter()
{
    init();
    std::cout<<"MyVideoWriter()"<<std::endl;
}

MyVideoWriter::~MyVideoWriter()
{
    closeWriteVideo();
    std::cout<<"~MyVideoWriter()"<<std::endl;
}


//初始化保存数据参数
int MyVideoWriter::initSaveVideoData(const std::string &fileName,cv::Size size,double fps,int codec,bool isColor)
{
    if(outputVideo.isOpened())
    {
        outputVideo.release();
    }
    if(outputVideo.open(fileName,codec,fps,size,isColor))
    {
        return 0;
    }
    else
    {
        std::cout<<"初始化保存视频对象发生错误"<<std::endl;
        return -20;
    }
}

//初始化保存数据参数
int MyVideoWriter::initSaveVideoData(const std::string &fileName, cv::Size size, double fps, bool isColor)
{
    if(outputVideo.isOpened())
    {
        outputVideo.release();
    }
    if(outputVideo.open(fileName,-1,fps,size,isColor))
    {
        return 0;
    }
    else
    {
        std::cout<<"初始化保存视频对象发生错误"<<std::endl;
        return -20;
    }
}

//关闭视频写入
void MyVideoWriter::closeWriteVideo()
{
    if(outputVideo.isOpened())
    {
        outputVideo.release();
    }
}

//得到写入视频的帧字节数
int MyVideoWriter::getWriteFrameBytes()
{
    int bytes=0;
    if(outputVideo.isOpened())
    {
        bytes=static_cast<int>(outputVideo.get(cv::VIDEOWRITER_PROP_FRAMEBYTES));
    }
    return bytes;
}

//保存视频文件
int MyVideoWriter::saveVideo(cv::Mat inFrame)
{
    if(inFrame.empty())
    {
        return 0;
    }
    if(outputVideo.isOpened())
    {
        outputVideo.write(inFrame);
        return 0;
    }
    else
    {
        std::cout<<"写入视频数据发生错误"<<std::endl;
        return -21;
    }
}

void MyVideoWriter::init()
{

}
