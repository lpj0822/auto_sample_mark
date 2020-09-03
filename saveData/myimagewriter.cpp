#include "myimagewriter.h"
#include <iostream>

MyImageWriter::MyImageWriter()
{
    std::cout << "MyImageWriter()" << std::endl;
}

MyImageWriter::~MyImageWriter()
{
    std::cout << "~MyImageWriter()" << std::endl;
}

//基于opencv的图片保存png
int MyImageWriter::saveImage(const cv::Mat &frame, const std::string &fileNamePath)
{
    if (frame.empty())
    {
        return 0;
    }

    if(!cv::imwrite(fileNamePath, frame))//保存图片
    {
        std::cout << "imwrite fail!" << std::endl;
        return -10;
    }
    return 0;
}
