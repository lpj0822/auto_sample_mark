#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QCursor>

#define PCL_NO_PRECOMPILE

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cfloat>
#include <pcl/visualization/eigen.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
#include <pcl/visualization/pcl_plotter.h>
#endif
#include <pcl/visualization/point_picking_event.h>
#include <pcl/search/kdtree.h>
#include <vtkPolyDataReader.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#ifdef WIN32
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
#endif
#include <QVTKWidget.h>

#include "dataType/myobject.h"
#include "baseAlgorithm/common_transform.h"

class PCLViewer : public QVTKWidget
{
    Q_OBJECT

public:

    typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
    typedef ColorHandler::Ptr ColorHandlerPtr;
    typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

    typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
    typedef GeometryHandler::Ptr GeometryHandlerPtr;
    typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

    PCLViewer(QWidget *parent = 0);
    ~PCLViewer();

    void setIsSelect(bool isSelect);
    int setNewPointCloud(const QString &pcdFilePath);
    void setOjects(const QList<MyObject> &obejcts, QString sampleClass);
    void clearPoints();
    void getPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &result);

protected:
    void enterEvent(QEvent *e);
    void leaveEvent(QEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *e);
    void clickedPointCallback(const pcl::visualization::PointPickingEvent& event);

private:

    void drawRandomColorPointCloud();
    void drawRGBPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    void drawShape(const QList<MyObject> &obejcts);
    void drawObject(const MyObject &object, int id);

    void initData();
    void initConnect();
    void initCloud();

private:
    pcl::PCDReader pcd;
    ColorHandlerPtr colorHandler;
    GeometryHandlerPtr geometryHandler;
    pcl::visualization::PCLVisualizer::Ptr viewer;

    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCLPointCloud2::Ptr srcCloud;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clickedPoints;

    pcl::search::KdTree<pcl::PointXYZ> search;

    QString sampleClass;

    QCursor myDrawCursor;
    bool isSelect;

    Transform transform;
};

#endif // PCLVIEWER_H
