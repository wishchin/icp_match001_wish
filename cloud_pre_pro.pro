QT -= gui

#QMAKE_CFLAGS_lSYSTEM =-l
CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000#disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp

HEADERS += include/cloud_pro.h
SOURCES += src/cloud_pro.cpp

INCLUDEPATH +=  /opt/ros/kinetic/include
INCLUDEPATH +=  /usr/include/pcl-1.7
INCLUDEPATH +=  /usr/include/eigen3
INCLUDEPATH +=  /usr/include/vtk-6.2

LIBS += -L/opt/ros/kinetic/lib/ -lcv_bridge -lmessage_filters -lz

LIBS += -L/opt/ros/kinetic/lib \
        -lroscpp \
        -lrospack \
        -lpthread \
        -lrosconsole \
        -lrosconsole_log4cxx \
        -lrosconsole_backend_interface \
        -lxmlrpcpp \
        -lroscpp_serialization \
        -lrostime \
        -lcpp_common \
        -lroslib \
        -ltf \
        -lyaml-cpp \
        -lkdl_conversions \
        -lcv_bridge \
        -lboost_system -lz
#LIBS += /usr/lib/x86_64-linux-gnu/libGLEW.so
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lpcl_visualization -lpcl_io -lpcl_surface -lpcl_common -lz
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lpcl_filters -lpcl_features -lz
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lconsole_bridge -lz
#LIBS += -L/usr/lib/x86_64-linux-gnu/ -lvtkRenderingCore-6.3 lvtkCommonDataModel-6.3 lvtkCommonCore-6.3 -lz
#LIBS += -L/usr/lib/x86_64-linux-gnu/ -lvtkGUISupportQt-6.3 lvtkGUISupportQtOpenGL-6.3 lvtkCommonMath-6.3 -lz
#LIBS += /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so
#LIBS += /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so
#LIBS += /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so
#LIBS += /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.3.so
#LIBS += /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.3.so
#LIBS += /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so
#LIBS += /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.3.so
LIBS += /usr/lib/x86_64-linux-gnu/libpcl_search.so
LIBS += /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so
LIBS += /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so
LIBS += /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so
LIBS += /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.2.so
LIBS += /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.2.so
LIBS += /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so
LIBS += /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so

#LIBS += /usr/lib/x86_64-linux-gnu/libopencv_core.so
#LIBS += /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
#LIBS += /usr/lib/x86_64-linux-gnu/libopencv_highgui.so
#LIBS += /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so
LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lz
LIBS += -L/usr/local/lib/ -lopencv_ml -lopencv_calib3d -lz
LIBS += -L/usr/local/lib/ -lopencv_features2d -lopencv_flann -lz
#LIBS += -L/usr/local/lib/ -lopencv_contrib -lz
LIBS += -L/usr/local/lib/ -lopencv_imgcodecs -lz
