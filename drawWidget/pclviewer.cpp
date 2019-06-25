#pragma execution_character_set("utf-8")
#include "pclviewer.h"
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <boost/bind.hpp>

PCLViewer::PCLViewer(QWidget *parent) : QVTKWidget(parent)
{
    initData();
    initConnect();
}

PCLViewer::~PCLViewer()
{

}

void PCLViewer::setIsSelect(bool isSelect)
{
    this->isSelect = isSelect;
}

void PCLViewer::setNewPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    float scale = 5;
    srcCloud->clear();
    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      PointT point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      point.r = static_cast<unsigned char>(cloud->points[i].intensity * scale);
      point.g = static_cast<unsigned char>(cloud->points[i].intensity * scale);
      point.b = static_cast<unsigned char>(cloud->points[i].intensity * scale);
      srcCloud->push_back(point);
    }
    viewer->updatePointCloud(srcCloud, "srcCloud");
    this->update();
}

void PCLViewer::clearPoints()
{
    if(clickedPoints->size() > 0)
    {
        clickedPoints->clear();
        viewer->updatePointCloud(clickedPoints, "clickedPoints");
        this->update();
    }
}

void PCLViewer::getPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &result)
{
    result->clear();
    for(const PointT &point: clickedPoints->points)
    {
        pcl::PointXYZI tempPoint;
        tempPoint.x = point.x;
        tempPoint.y = point.y;
        tempPoint.z = point.z;
        tempPoint.intensity = 0;
        result->push_back(tempPoint);
    }
}

void PCLViewer::enterEvent(QEvent *e)
{
    if(isSelect)
    {
        setCursor(myDrawCursor);
    }
    QVTKWidget::enterEvent(e);
}

void PCLViewer::leaveEvent(QEvent *e)
{
    QVTKWidget::leaveEvent(e);
}

void PCLViewer::mouseDoubleClickEvent(QMouseEvent *e)
{
    QVTKWidget::mouseDoubleClickEvent(e);
}

void PCLViewer::clickedPointCallback(const pcl::visualization::PointPickingEvent& event)
{
    if(isSelect)
    {
        bool isIn = false;
        PointT currentPoint;
        if (event.getPointIndex() == -1)
            return;
        event.getPoint(currentPoint.x, currentPoint.y, currentPoint.z);
        currentPoint.r = 0;
        currentPoint.g = 0;
        currentPoint.b = 0;
        for(size_t loop = 0; loop < this->clickedPoints->size(); loop++)
        {
            if(this->clickedPoints->points[loop].x != currentPoint.x
                    && this->clickedPoints->points[loop].y != currentPoint.y
                    && this->clickedPoints->points[loop].z != currentPoint.z)
            {
                isIn = true;
                break;
            }
        }
        if(isIn || this->clickedPoints->size() <= 0)
            this->clickedPoints->push_back(currentPoint);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(clickedPoints, 255, 0, 0);

        this->viewer->removePointCloud("clickedPoints");
        this->viewer->addPointCloud(this->clickedPoints, red, "clickedPoints");
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clickedPoints");

        this->update();

        std::cout << "point:" << currentPoint.x << " " << currentPoint.y << " " << currentPoint.z << std::endl;
    }
}

void PCLViewer::initData()
{
    viewer.reset (new pcl::visualization::PCLVisualizer("cloud", false));
    initCloud();

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> colorHandler(srcCloud);
    viewer->addPointCloud<PointT>(srcCloud, colorHandler, "srcCloud", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);

    this->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor (), this->GetRenderWindow ());
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    viewer->registerPointPickingCallback(boost::bind(&PCLViewer::clickedPointCallback, this, _1));

    this->update();

    isSelect = false;
    myDrawCursor = QCursor(QPixmap(tr(":/images/images/cross.png")));
}

void PCLViewer::initConnect()
{

}

void PCLViewer::initCloud()
{
    // Setup the cloud pointer
    srcCloud.reset (new PointCloudT);
    // The number of points in the cloud
    srcCloud->points.resize(10);

    // Fill the cloud with some points
    for (size_t i = 0; i < srcCloud->points.size (); ++i)
    {
        srcCloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        srcCloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        srcCloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

        srcCloud->points[i].r = static_cast<unsigned char>(255 *(1024 * rand() / (RAND_MAX + 1.0f)));
        srcCloud->points[i].g = static_cast<unsigned char>(255 *(1024 * rand() / (RAND_MAX + 1.0f)));
        srcCloud->points[i].b = static_cast<unsigned char>(255 *(1024 * rand() / (RAND_MAX + 1.0f)));
    }

    clickedPoints.reset(new PointCloudT);
}
