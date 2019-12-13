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

int PCLViewer::setNewPointCloud(const QString &pcdFilePath)
{
    int version;
//    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZI>);
//    if(pcl::io::loadPCDFile(pcdFilePath.toStdString(), *currentCloud) == -1)
//    {
//        return -1;
//    }
    srcCloud.reset (new pcl::PCLPointCloud2);
    if(pcd.read(pcdFilePath.toStdString(), *srcCloud, origin, orientation, version) < 0)
    {
        return -1;
    }
    drawRandomColorPointCloud();
    return 0;
}

void PCLViewer::setOjects(const QList<MyObject> &obejcts, QString sampleClass)
{
    QList<MyObject> rect3DObejcts;
    rect3DObejcts.clear();
    for(int loop = 0; loop < obejcts.count(); loop++)
    {
        const MyObject object = obejcts[loop];
        if(object.getShapeType() == ShapeType::RECT3D_SHAPE)
        {
            rect3DObejcts.append(object);
        }
    }
    this->sampleClass = sampleClass;
    drawShape(rect3DObejcts);
}

void PCLViewer::clearPoints()
{
    viewer->removeAllShapes();
    viewer->removePointCloud("srcCloud");
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
    for(const pcl::PointXYZRGB &point: clickedPoints->points)
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
        pcl::PointXYZRGB currentPoint;
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

void PCLViewer::drawRandomColorPointCloud()
{
    viewer->removePointCloud("srcCloud");
    colorHandler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(srcCloud));
    geometryHandler.reset(new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(srcCloud));
    //geometryHandler.reset(new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2>(srcCloud));
    viewer->addPointCloud(srcCloud, geometryHandler, colorHandler, origin, orientation, "srcCloud", 0);
}

void PCLViewer::drawRGBPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    float scale = 5;
    rgbCloud->clear();
    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      pcl::PointXYZRGB point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      point.r = static_cast<unsigned char>(cloud->points[i].intensity * scale);
      point.g = static_cast<unsigned char>(cloud->points[i].intensity * scale);
      point.b = static_cast<unsigned char>(cloud->points[i].intensity * scale);
      rgbCloud->push_back(point);
    }
    viewer->updatePointCloud(rgbCloud, "srcCloud");
    this->update();
}

void PCLViewer::drawShape(const QList<MyObject> &obejcts)
{
    viewer->removeAllShapes();
    for(int loop = 0; loop < obejcts.count(); loop++)
    {
        if(sampleClass == "All")
        {
            drawObject(obejcts[loop], loop);
        }
        else
        {
            if(obejcts[loop].getObjectClass().contains(sampleClass))
            {
                drawObject(obejcts[loop], loop);
            }
        }
    }
    this->update();
}

void PCLViewer::drawObject(const MyObject &object, int id)
{
    pcl::PointXYZ point;
    const MyRect3D rect3d = object.getBox3D();
    QString boxId = QString("box_%1").arg(id);
    Rotation yaw = transform.getRotation(0, 0, rect3d.theta);
    double length = static_cast<double>(rect3d.size[0]);
    double width = static_cast<double>(rect3d.size[1]);
    double height = static_cast<double>(rect3d.size[2]);
    point.x = rect3d.center[0];
    point.y = rect3d.center[1];
    point.z = rect3d.center[2] + static_cast<float>(height / 2.0);
    viewer->addText3D<pcl::PointXYZ>(object.getObjectClass().toStdString(), point, 1.0, \
                                     255, 255, 0, QString("text_%1").arg(id).toStdString());
    viewer->addCube(rect3d.center, yaw, length, width, height, boxId.toStdString());
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, \
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, boxId.toStdString());
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, boxId.toStdString());
}

void PCLViewer::initData()
{
    viewer.reset (new pcl::visualization::PCLVisualizer("cloud", false));
    initCloud();

    colorHandler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(srcCloud));
    geometryHandler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(srcCloud));
    viewer->addPointCloud (srcCloud, geometryHandler, colorHandler, origin, orientation, "srcCloud", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, "srcCloud");

    this->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor (), this->GetRenderWindow ());

    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->cameraParamsSet ();
    viewer->cameraFileLoaded ();
    viewer->resetCameraViewpoint("srcCloud");
    viewer->resetCamera();

    viewer->registerPointPickingCallback(boost::bind(&PCLViewer::clickedPointCallback, this, _1));

    // Set whether or not we should be using the vtkVertexBufferObjectMapper
    viewer->setUseVbos(true);

    this->update();

    sampleClass = "All";

    isSelect = false;
    myDrawCursor = QCursor(QPixmap(tr(":/images/images/cross.png")));
}

void PCLViewer::initConnect()
{

}

void PCLViewer::initCloud()
{
    srcCloud.reset (new pcl::PCLPointCloud2);
    clickedPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}
