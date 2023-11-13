QT       += core gui serialport multimedia multimediawidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src\cameracalibration.cpp \
    src\yamlhelp.cpp \
    src\lidarcalibration.cpp \
    src\mainwindow.cpp \
    camera\car_cam_recv.cpp \
    main.cpp \
    thirdparty\json\json11.cpp

HEADERS += \
    core/debug.hpp \
    core/def.hpp \
    core/def_end.hpp \
    core/expected.hpp \
    core/fs.hpp \
    core/future.hpp \
    core/memory.hpp \
    core/result.hpp \
    include\cameracalibration.h \
    include\define_type.h \
    include\yamlhelp.h \
    include\lidarcalibration.h \
    include\mainwindow.h \
    camera\car_cam_recv.hpp \
    camera\byte_queue.h \
    frame_rc.hpp

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += $$PWD\core
INCLUDEPATH += $$PWD\include
INCLUDEPATH += $$PWD\thirdparty
INCLUDEPATH += $$PWD\thirdparty\asio-1.22.1

LIBS += -lws2_32
INCLUDEPATH += $$PWD\thirdparty\yaml-cpp\include

CONFIG(debug, debug|release):{
LIBS += -L$$PWD\thirdparty\yaml-cpp\lib \
    -lyaml-cppd
} else:CONFIG(release, debug|release):{
LIBS += -L$$PWD\thirdparty\yaml-cpp\lib \
    -lyaml-cpp
}

INCLUDEPATH += $$PWD\deps\opencv\include


CONFIG(debug, debug|release):{
LIBS += -L$$PWD\deps\opencv\lib \
    -lopencv_core480d \
    -lopencv_highgui480d \
    -lopencv_imgcodecs480d \
    -lopencv_imgproc480d \
    -lopencv_features2d480d \
    -lopencv_video480d \
    -lopencv_videoio480d \
    -lopencv_calib3d480d
} else:CONFIG(release, debug|release):{
LIBS += -L$$PWD\deps\opencv\lib \
    -lopencv_core480 \
    -lopencv_highgui480 \
    -lopencv_imgcodecs480 \
    -lopencv_imgproc480 \
    -lopencv_features2d480 \
    -lopencv_video480 \
    -lopencv_videoio480 \
    -lopencv_calib3d480
}

DISTFILES += \
    config.yaml




