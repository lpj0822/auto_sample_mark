T#-------------------------------------------------
#
# Project created by QtCreator 2017-05-08T12:15:54
#
#-------------------------------------------------

QT       += core gui xml
QT       += multimedia
QT       += multimediawidgets
CONFIG   += c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SampleMark
TEMPLATE = app

include(helpers/helpers.pri)
include(utilityGUI/utilityGUI.pri)
include(saveData/saveData.pri)
include(baseAlgorithm/baseAlgorithm.pri)
include(deepLearning/deepLearning.pri)
include(multipletracking/multipletracking.pri)

SOURCES += main.cpp\
        controlwindow.cpp \
    markclasswindow.cpp \
    mainwindow.cpp \
    selectmarkclasswindow.cpp \
    autoSampleMark/autoparamterconfig.cpp \
    autoSampleMark/autosamplemarkthread.cpp \
    autoSampleMark/autosamplemarkwindow.cpp \
    videoTools/croppingvideothread.cpp \
    videoTools/fromvideotopicturewindow.cpp \
    videoTools/videocroppingwindow.cpp \
    videoTools/videocuttingwindow.cpp \
    videoTools/frompicturetovideowindow.cpp \
    autoSampleMark/modellabelwindow.cpp \
    autoSampleMark/autoparamterconfigwindow.cpp \
    drawShape/myshape.cpp \
    drawShape/drawrectshape.cpp \
    drawShape/drawlineshape.cpp \
    saveMarkData/jsonprocess.cpp \
    saveMarkData/xmlprocess.cpp \
    drawShape/drawpolygonshape.cpp \
    saveMarkData/jsonprocessvideo.cpp \
    videoTools/qcamerawindow.cpp \
    videomultipletracking.cpp \
    imagecontrolwindow.cpp \
    videocontrolwindow.cpp \
    drawWidget/editablelabel.cpp \
    drawWidget/pclviewer.cpp \
    pclcontrolwindow.cpp \
    sampleMarkParam/manualparamterconfig.cpp \
    sampleMarkParam/videomarkparamterconfig.cpp \
    sampleMarkParam/segmentparamterconfig.cpp \
    videoTools/imageconverterthread.cpp \
    videoTools/imageconverterwindow.cpp \
    drawShape/drawlaneshape.cpp \
    drawShape/drawshape.cpp \
    imagesegmentcontrolwindow.cpp \
    drawWidget/segmentlabel.cpp \
    saveMarkData/segmentimagesave.cpp \
    dataType/myobject.cpp \
    dataType/myrect3d.cpp \
    drawShape/segmentshape.cpp \
    paramWindow/manualparamterconfigwindow.cpp \
    paramWindow/segmentparamterconfigwindow.cpp \
    paramWindow/videoparameterwindow.cpp \
    paramWindow/videomarkparamterwindow.cpp \
    pcTools/pcdconverterthread.cpp \
    pcTools/pcdconverterwindow.cpp \
    sampleMarkParam/pointcloudparamterconfig.cpp \
    paramWindow/pointcloudmarkparamterwindow.cpp

HEADERS  += controlwindow.h \
    markclasswindow.h \
    mainwindow.h \
    selectmarkclasswindow.h \
    autoSampleMark/autoparamterconfig.h \
    autoSampleMark/autosamplemarkthread.h \
    autoSampleMark/autosamplemarkwindow.h \
    videoTools/croppingvideothread.h \
    videoTools/fromvideotopicturewindow.h \
    videoTools/videocroppingwindow.h \
    videoTools/videocuttingwindow.h \
    videoTools/frompicturetovideowindow.h \
    autoSampleMark/modellabelwindow.h \
    autoSampleMark/autoparamterconfigwindow.h \
    drawShape/myshape.h \
    drawShape/drawrectshape.h \
    drawShape/drawlineshape.h \
    saveMarkData/jsonprocess.h \
    saveMarkData/xmlprocess.h \
    drawShape/drawpolygonshape.h \
    saveMarkData/jsonprocessvideo.h \
    videoTools/qcamerawindow.h \
    videomultipletracking.h \
    imagecontrolwindow.h \
    videocontrolwindow.h \
    drawWidget/editablelabel.h \
    drawWidget/pclviewer.h \
    pclcontrolwindow.h \
    sampleMarkParam/drawmarkcolor.h \
    sampleMarkParam/manualparamterconfig.h \
    sampleMarkParam/videomarkparamterconfig.h \
    sampleMarkParam/segmentparamterconfig.h \
    videoTools/imageconverterthread.h \
    videoTools/imageconverterwindow.h \
    drawShape/drawlaneshape.h \
    drawShape/drawshape.h \
    imagesegmentcontrolwindow.h \
    drawWidget/segmentlabel.h \
    saveMarkData/segmentimagesave.h \
    dataType/myobject.h \
    dataType/myrect3d.h \
    drawShape/segmentshape.h \
    paramWindow/manualparamterconfigwindow.h \
    paramWindow/segmentparamterconfigwindow.h \
    paramWindow/videoparameterwindow.h \
    paramWindow/videomarkparamterwindow.h \
    pcTools/pcdconverterthread.h \
    pcTools/pcdconverterwindow.h \
    sampleMarkParam/pointcloudparamterconfig.h \
    paramWindow/pointcloudmarkparamterwindow.h

RESOURCES += \
    style.qrc \
    images.qrc \
    QtAwesome.qrc \
    document.qrc \
    qm.qrc

RC_ICONS = appico.ico

TRANSLATIONS = zh.ts en.ts ja.ts

win32{

win32:DEFINES+=_CRT_SECURE_NO_WARNINGS

INCLUDEPATH+= D:\opencv\opencv400\MyBuild\install\include

INCLUDEPATH+= D:\PCL_1_8_1\3rdParty\Boost\include\boost-1_64
INCLUDEPATH+= D:\PCL_1_8_1\3rdParty\Eigen\eigen3
INCLUDEPATH+= D:\PCL_1_8_1\3rdParty\FLANN\include
INCLUDEPATH+= D:\PCL_1_8_1\3rdParty\Qhull\include
INCLUDEPATH+= D:\PCL_1_8_1\3rdParty\OpenNI2\Include
INCLUDEPATH+= D:\PCL_1_8_1\3rdParty\VTK\include\vtk-8.0
INCLUDEPATH+= D:\PCL_1_8_1\include\pcl-1.8

CONFIG(debug,debug|release){
LIBS += D:\PCL_1_8_1\lib\pcl_common_debug.lib\
        D:\PCL_1_8_1\lib\pcl_features_debug.lib\
        D:\PCL_1_8_1\lib\pcl_filters_debug.lib\
        D:\PCL_1_8_1\lib\pcl_io_debug.lib\
        D:\PCL_1_8_1\lib\pcl_io_ply_debug.lib\
        D:\PCL_1_8_1\lib\pcl_kdtree_debug.lib\
        D:\PCL_1_8_1\lib\pcl_keypoints_debug.lib\
        D:\PCL_1_8_1\lib\pcl_ml_debug.lib\
        D:\PCL_1_8_1\lib\pcl_octree_debug.lib\
        D:\PCL_1_8_1\lib\pcl_outofcore_debug.lib\
        D:\PCL_1_8_1\lib\pcl_people_debug.lib\
        D:\PCL_1_8_1\lib\pcl_recognition_debug.lib\
        D:\PCL_1_8_1\lib\pcl_registration_debug.lib\
        D:\PCL_1_8_1\lib\pcl_sample_consensus_debug.lib\
        D:\PCL_1_8_1\lib\pcl_search_debug.lib\
        D:\PCL_1_8_1\lib\pcl_segmentation_debug.lib\
        D:\PCL_1_8_1\lib\pcl_stereo_debug.lib\
        D:\PCL_1_8_1\lib\pcl_surface_debug.lib\
        D:\PCL_1_8_1\lib\pcl_tracking_debug.lib\
        D:\PCL_1_8_1\lib\pcl_visualization_debug.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_atomic-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_chrono-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_container-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_context-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_coroutine-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_date_time-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_exception-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_filesystem-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_graph-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_iostreams-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_locale-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_log_setup-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_log-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_c99f-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_c99l-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_c99-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_tr1f-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_tr1l-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_tr1-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_mpi-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_prg_exec_monitor-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_program_options-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_random-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_regex-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_serialization-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_signals-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_system-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_test_exec_monitor-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_thread-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_timer-vc140-mt-gd-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_type_erasure-vc140-mt-gd-1_64.lib
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_unit_test_framework-vc140-mt-gd-1_64.lib
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_wave-vc140-mt-gd-1_64.lib
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_wserialization-vc140-mt-gd-1_64.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\FLANN\lib\flann_cpp_s-gd.lib\
        D:\PCL_1_8_1\3rdParty\FLANN\lib\flann_s-gd.lib\
        D:\PCL_1_8_1\3rdParty\FLANN\lib\flann-gd.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\Qhull\lib\qhullstatic_d.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhull_d.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhull_p_d.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhullcpp_d.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhullstatic_r_d.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\VTK\lib\vtkalglib-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkChartsCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonColor-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonComputationalGeometry-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonDataModel-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonExecutionModel-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonMath-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonMisc-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonSystem-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonTransforms-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkDICOMParser-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkDomainsChemistry-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkexoIIc-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkexpat-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersAMR-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersExtraction-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersFlowPaths-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersGeneral-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersGeneric-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersGeometry-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersHybrid-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersHyperTree-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersImaging-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersModeling-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersParallel-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersParallelImaging-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersProgrammable-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersSelection-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersSMP-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersSources-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersStatistics-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersTexture-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersVerdict-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkfreetype-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGeovisCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkgl2ps-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGUISupportQt-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGUISupportQtOpenGL-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGUISupportQtSQL-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkhdf5-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkhdf5_hl-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingColor-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingFourier-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingGeneral-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingHybrid-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingMath-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingMorphological-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingSources-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingStatistics-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingStencil-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInfovisCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInfovisLayout-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInteractionImage-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInteractionStyle-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInteractionWidgets-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOAMR-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOEnSight-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOExodus-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOExport-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOGeometry-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOImage-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOImport-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOInfovis-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOLegacy-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOLSDyna-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOMINC-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOMovie-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIONetCDF-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOParallel-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOParallelXML-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOPLY-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOSQL-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOVideo-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOXML-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOXMLParser-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkjpeg-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkjsoncpp-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtklibxml2-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkmetaio-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkNetCDF-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkoggtheora-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkParallelCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkpng-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkproj4-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingAnnotation-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingContext2D-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingContextOpenGL-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingFreeType-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingGL2PS-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingImage-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingLabel-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingLIC-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingLOD-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingOpenGL-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingVolume-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingVolumeOpenGL-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtksqlite-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtksys-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtktiff-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkverdict-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkViewsContext2D-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkViewsCore-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkViewsInfovis-8.0-gd.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkzlib-8.0-gd.lib

LIBS+=  D:\PCL_1_8_1\3rdParty\OpenNI2\Lib\OpenNI2.lib

LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xphoto400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xobjdetect400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ximgproc400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xfeatures2d400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videostab400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videoio400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_video400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_tracking400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_text400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_surface_matching400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_superres400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_structured_light400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stitching400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stereo400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_shape400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_saliency400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_rgbd400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_reg400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_plot400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_photo400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_phase_unwrapping400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_optflow400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_objdetect400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ml400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_line_descriptor400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgproc400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgcodecs400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_img_hash400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_highgui400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hfs400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hdf400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_fuzzy400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_flann400d.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_features2d400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_face400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dpm400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn_objdetect400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_datasets400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_core400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ccalib400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_calib3d400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bioinspired400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bgsegm400d.lib
LIBS+= D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_aruco400d.lib

} else {

LIBS += D:\PCL_1_8_1\lib\pcl_common_release.lib\
        D:\PCL_1_8_1\lib\pcl_features_release.lib\
        D:\PCL_1_8_1\lib\pcl_filters_release.lib\
        D:\PCL_1_8_1\lib\pcl_io_release.lib\
        D:\PCL_1_8_1\lib\pcl_io_ply_release.lib\
        D:\PCL_1_8_1\lib\pcl_kdtree_release.lib\
        D:\PCL_1_8_1\lib\pcl_keypoints_release.lib\
        D:\PCL_1_8_1\lib\pcl_ml_release.lib\
        D:\PCL_1_8_1\lib\pcl_octree_release.lib\
        D:\PCL_1_8_1\lib\pcl_outofcore_release.lib\
        D:\PCL_1_8_1\lib\pcl_people_release.lib\
        D:\PCL_1_8_1\lib\pcl_recognition_release.lib\
        D:\PCL_1_8_1\lib\pcl_registration_release.lib\
        D:\PCL_1_8_1\lib\pcl_sample_consensus_release.lib\
        D:\PCL_1_8_1\lib\pcl_search_release.lib\
        D:\PCL_1_8_1\lib\pcl_segmentation_release.lib\
        D:\PCL_1_8_1\lib\pcl_stereo_release.lib\
        D:\PCL_1_8_1\lib\pcl_surface_release.lib\
        D:\PCL_1_8_1\lib\pcl_tracking_release.lib\
        D:\PCL_1_8_1\lib\pcl_visualization_release.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_atomic-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_chrono-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_container-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_context-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_coroutine-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_date_time-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_exception-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_filesystem-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_graph-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_iostreams-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_locale-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_log_setup-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_log-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_c99f-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_c99l-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_c99-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_tr1f-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_tr1l-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_math_tr1-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_mpi-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_prg_exec_monitor-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_program_options-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_random-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_regex-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_serialization-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_signals-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_system-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_test_exec_monitor-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_thread-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_timer-vc140-mt-1_64.lib\
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_type_erasure-vc140-mt-1_64.lib
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_unit_test_framework-vc140-mt-1_64.lib
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_wave-vc140-mt-1_64.lib
        D:\PCL_1_8_1\3rdParty\Boost\lib\libboost_wserialization-vc140-mt-1_64.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\FLANN\lib\flann_cpp_s.lib\
        D:\PCL_1_8_1\3rdParty\FLANN\lib\flann_s.lib\
        D:\PCL_1_8_1\3rdParty\FLANN\lib\flann.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\Qhull\lib\qhullstatic_d.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhull.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhull_p.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhullcpp.lib\
        D:\PCL_1_8_1\3rdParty\Qhull\lib\qhullstatic_r.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\VTK\lib\vtkalglib-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkChartsCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonColor-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonComputationalGeometry-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonDataModel-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonExecutionModel-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonMath-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonMisc-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonSystem-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkCommonTransforms-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkDICOMParser-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkDomainsChemistry-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkexoIIc-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkexpat-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersAMR-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersExtraction-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersFlowPaths-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersGeneral-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersGeneric-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersGeometry-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersHybrid-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersHyperTree-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersImaging-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersModeling-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersParallel-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersParallelImaging-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersProgrammable-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersSelection-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersSMP-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersSources-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersStatistics-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersTexture-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkFiltersVerdict-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkfreetype-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGeovisCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkgl2ps-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGUISupportQt-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGUISupportQtOpenGL-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkGUISupportQtSQL-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkhdf5-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkhdf5_hl-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingColor-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingFourier-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingGeneral-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingHybrid-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingMath-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingMorphological-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingSources-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingStatistics-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkImagingStencil-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInfovisCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInfovisLayout-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInteractionImage-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInteractionStyle-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkInteractionWidgets-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOAMR-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOEnSight-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOExodus-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOExport-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOGeometry-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOImage-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOImport-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOInfovis-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOLegacy-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOLSDyna-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOMINC-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOMovie-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIONetCDF-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOParallel-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOParallelXML-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOPLY-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOSQL-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOVideo-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOXML-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkIOXMLParser-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkjpeg-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkjsoncpp-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtklibxml2-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkmetaio-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkNetCDF-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkoggtheora-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkParallelCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkpng-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkproj4-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingAnnotation-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingContext2D-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingContextOpenGL-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingFreeType-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingGL2PS-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingImage-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingLabel-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingLIC-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingLOD-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingOpenGL-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingVolume-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkRenderingVolumeOpenGL-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtksqlite-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtksys-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtktiff-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkverdict-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkViewsContext2D-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkViewsCore-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkViewsInfovis-8.0.lib\
        D:\PCL_1_8_1\3rdParty\VTK\lib\vtkzlib-8.0.lib
LIBS+=  D:\PCL_1_8_1\3rdParty\OpenNI2\Lib\OpenNI2.lib

LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xphoto400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xobjdetect400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ximgproc400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xfeatures2d400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videostab400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videoio400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_video400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_tracking400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_text400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_surface_matching400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_superres400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_structured_light400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stitching400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stereo400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_shape400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_saliency400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_rgbd400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_reg400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_plot400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_photo400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_phase_unwrapping400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_optflow400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_objdetect400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ml400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_line_descriptor400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgproc400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgcodecs400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_img_hash400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_highgui400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hfs400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hdf400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_fuzzy400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_flann400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_features2d400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_face400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dpm400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn_objdetect400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_datasets400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_core400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ccalib400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_calib3d400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bioinspired400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bgsegm400.lib
LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_aruco400.lib
}
}

unix{

INCLUDEPATH+= /home/lpj/Software/opencv34/include

LIBS+=-L/home/lpj/Software/opencv34/lib \
    -lopencv_stitching \
    -lopencv_superres \
    -lopencv_cudacodec \
    -lopencv_videostab \
    -lopencv_bgsegm \
    -lopencv_bioinspired \
    -lopencv_ccalib \
    -lopencv_dnn_objdetect \
    -lopencv_dpm \
    -lopencv_face \
    -lopencv_freetype \
    -lopencv_fuzzy \
    -lopencv_hdf \
    -lopencv_hfs \
    -lopencv_img_hash \
    -lopencv_line_descriptor \
    -lopencv_optflow \
    -lopencv_reg \
    -lopencv_rgbd \
    -lopencv_saliency \
    -lopencv_sfm \
    -lopencv_stereo \
    -lopencv_structured_light \
    -lopencv_viz \
    -lopencv_phase_unwrapping \
    -lopencv_surface_matching \
    -lopencv_tracking \
    -lopencv_datasets \
    -lopencv_text \
    -lopencv_dnn \
    -lopencv_plot \
    -lopencv_xfeatures2d \
    -lopencv_shape \
    -lopencv_video \
    -lopencv_ml \
    -lopencv_ximgproc \
    -lopencv_xobjdetect \
    -lopencv_objdetect \
    -lopencv_calib3d \
    -lopencv_features2d \
    -lopencv_highgui \
    -lopencv_videoio \
    -lopencv_imgcodecs \
    -lopencv_flann \
    -lopencv_xphoto \
    -lopencv_photo \
    -lopencv_imgproc \
    -lopencv_core

#Eigen
INCLUDEPATH += /usr/include/eigen3

#Boost
INCLUDEPATH += /usr/include/boost
LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so

#VTK
INCLUDEPATH += /home/lpj/Software/vtk_8_1_2/include/vtk-8.1
LIBS += /home/lpj/Software/vtk_8_1_2/lib/libvtk*.so

#PCL Header
INCLUDEPATH += /home/lpj/Software/pcl_1_9/include/pcl-1.9
LIBS += /home/lpj/Software/pcl_1_9/lib/libpcl_*.so
}
