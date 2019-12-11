#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QCursor>

#define PCL_NO_PRECOMPILE

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/visualization/point_picking_event.h>

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

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    PCLViewer(QWidget *parent = 0);
    ~PCLViewer();

    void setIsSelect(bool isSelect);
    void setNewPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
    void setOjects(const QList<MyObject> &obejcts, QString sampleClass);
    void clearPoints();
    void getPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &result);

protected:
    void enterEvent(QEvent *e);
    void leaveEvent(QEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *e);
    void clickedPointCallback(const pcl::visualization::PointPickingEvent& event);

private:

    void drawShape(const QList<MyObject> &obejcts);
    void drawObject(const MyObject &object, int id);

    void initData();
    void initConnect();
    void initCloud();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    PointCloudT::Ptr srcCloud;
    PointCloudT::Ptr clickedPoints;

    QString sampleClass;

    QCursor myDrawCursor;
    bool isSelect;

    Transform transform;
};

#endif // PCLVIEWER_H
