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
