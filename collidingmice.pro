QT += widgets

HEADERS += \
	mouse.h \
        Robot.h \
        CustomView.h
SOURCES += \
	main.cpp \
        mouse.cpp \
        Robot.cpp \
        CustomView.cpp

RESOURCES += \
	mice.qrc

QT += core
# install
target.path = $$[QT_INSTALL_EXAMPLES]/widgets/graphicsview/collidingmice
INSTALLS += target

CONFIG += c++11
