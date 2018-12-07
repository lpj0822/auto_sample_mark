#-------------------------------------------------
#
# Project created by QtCreator 2017-05-08T12:15:54
#
#-------------------------------------------------

QT       += core gui xml
QT       += multimedia
QT       += multimediawidgets
QT       += multimedia
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
    editablelabel.cpp \
    videoparameterwindow.cpp \
    manualparamterconfig.cpp \
    manualparamterconfigwindow.cpp \
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
    videomarkparamterwindow.cpp \
    videomarkparamterconfig.cpp \
    myobject.cpp \
    drawShape/myshape.cpp \
    drawShape/drawrectshape.cpp \
    drawShape/drawlineshape.cpp \
    saveMarkData/jsonprocess.cpp \
    saveMarkData/xmlprocess.cpp \
    drawShape/drawpolygonshape.cpp \
    saveMarkData/jsonprocessvideo.cpp \
    videoTools/qcamerawindow.cpp \
    videomultipletracking.cpp

HEADERS  += controlwindow.h \
    editablelabel.h \
    videoparameterwindow.h \
    manualparamterconfig.h \
    manualparamterconfigwindow.h \
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
    videomarkparamterwindow.h \
    videomarkparamterconfig.h \
    myobject.h \
    drawShape/myshape.h \
    drawShape/drawrectshape.h \
    drawShape/drawlineshape.h \
    saveMarkData/jsonprocess.h \
    saveMarkData/xmlprocess.h \
    drawShape/drawpolygonshape.h \
    saveMarkData/jsonprocessvideo.h \
    videoTools/qcamerawindow.h \
    videomultipletracking.h

RESOURCES += \
    style.qrc \
    images.qrc \
    QtAwesome.qrc \
    document.qrc

RC_ICONS = appico.ico

INCLUDEPATH+= D:\opencv\opencv320\MyBuild\install\include\
              D:\opencv\opencv320\MyBuild\install\include\opencv\
              D:\opencv\opencv320\MyBuild\install\include\opencv2

#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_aruco320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bgsegm320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bioinspired320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_calib3d320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ccalib320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_core320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_datasets320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dnn320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dpm320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_face320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_features2d320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_flann320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_fuzzy320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_highgui320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgcodecs320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgproc320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_line_descriptor320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ml320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_objdetect320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_optflow320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_phase_unwrapping320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_photo320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_plot320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_reg320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_rgbd320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_saliency320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_shape320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stereo320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stitching320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_structured_light320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_superres320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_surface_matching320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_text320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_tracking320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_video320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videoio320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videostab320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xfeatures2d320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ximgproc320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320d.lib
#LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320d.lib

LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_aruco320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bgsegm320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_bioinspired320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_calib3d320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ccalib320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_core320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_datasets320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dnn320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_dpm320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_face320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_features2d320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_flann320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_fuzzy320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_highgui320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgcodecs320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_imgproc320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_line_descriptor320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ml320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_objdetect320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_optflow320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_phase_unwrapping320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_photo320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_plot320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_reg320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_rgbd320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_saliency320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_shape320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stereo320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_stitching320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_structured_light320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_superres320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_surface_matching320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_text320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_tracking320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_video320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videoio320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_videostab320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xfeatures2d320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_ximgproc320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320.lib
LIBS+=D:\opencv\opencv320\MyBuild\install\x64\vc12\lib\opencv_xobjdetect320.lib
