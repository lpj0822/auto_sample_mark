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

INCLUDEPATH+= "C:/Program Files (x86)/Windows Kits/10/Include/10.0.10240.0/ucrt"
INCLUDEPATH+= D:\opencv\opencv400\MyBuild\install\include

LIBS+=-L"C:/Program Files (x86)/Windows Kits/10/Lib/10.0.10240.0/ucrt/x64"

#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xphoto400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xobjdetect400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ximgproc400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_xfeatures2d400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videostab400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_videoio400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_video400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_tracking400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_text400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_surface_matching400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_superres400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_structured_light400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stitching400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_stereo400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_shape400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_saliency400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_rgbd400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_reg400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_plot400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_photo400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_phase_unwrapping400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_optflow400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_objdetect400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ml400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_line_descriptor400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgproc400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_imgcodecs400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_img_hash400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_highgui400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hfs400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_hdf400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_fuzzy400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_flann400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_features2d400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_face400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dpm400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_dnn_objdetect400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_datasets400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_core400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_ccalib400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_calib3d400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bioinspired400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_bgsegm400d.lib
#LIBS+=D:\opencv\opencv400\MyBuild\install\x64\vc14\lib\opencv_aruco400d.lib

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
