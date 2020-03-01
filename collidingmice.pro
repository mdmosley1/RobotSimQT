# Note: when compiling on mac os x, run the following: 
# QMAKESPEC=macx-g++ qmake

QT += widgets

HEADERS += \
	mouse.h \
        Robot.h \
        CustomView.h\
        Waypoint.h\
        mainwindow.h\
        AprilTag.h\
        EstimatedPose.h\
        logging.h \
        KalmanFilter.h
SOURCES += \
	main.cpp \
        mouse.cpp \
        Robot.cpp \
        CustomView.cpp\
        Waypoint.cpp\
        mainwindow.cpp\
        AprilTag.cpp\
        EstimatedPose.cpp \
        logging.cpp \
        KalmanFilter.cpp


QT += core
# install
target.path = $$[QT_INSTALL_EXAMPLES]/widgets/graphicsview/collidingmice
INSTALLS += target

CONFIG += c++11

DEFINES += QCUSTOMPLOT_USE_LIBRARY

QCPLIB = qcustomplot

LIBS += -L./ -L/usr/local/lib/ -lqcustomplot -l$$QCPLIB -lyaml-cpp
LIBS +=  `pkg-config liblog4cxx --libs`

# include for log4cxx
QMAKE_CXXFLAGS += -I/usr/local/include/
