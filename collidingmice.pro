QT += widgets

HEADERS += \
	mouse.h \
        Robot.h \
        CustomView.h\
        Waypoint.h\
        mainwindow.h\
        AprilTag.h\
        EstimatedPose.h
SOURCES += \
	main.cpp \
        mouse.cpp \
        Robot.cpp \
        CustomView.cpp\
        Waypoint.cpp\
        mainwindow.cpp\
        AprilTag.cpp\
        EstimatedPose.cpp


QT += core
# install
target.path = $$[QT_INSTALL_EXAMPLES]/widgets/graphicsview/collidingmice
INSTALLS += target

CONFIG += c++11

DEFINES += QCUSTOMPLOT_USE_LIBRARY

QCPLIB = qcustomplot

LIBS += -L./ -L/usr/local/lib/libqcustomplot.so.2 -l$$QCPLIB  

