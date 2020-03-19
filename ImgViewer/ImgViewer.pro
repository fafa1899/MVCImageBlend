QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += console

QMAKE_CXXFLAGS+=/openmp

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
    geometryalg.cpp \
    main.cpp \
    mainwindow.cpp \
    qimageshowwidget.cpp

HEADERS += \
    geometryalg.h \
    mainwindow.h \
    qimageshowwidget.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += D:/SoftWare/OpenCV/opencv/build/include \
    D:/Work/CGALBuild/CGAL-4.14.3/include D:/SoftWare/boost_1_72_0 \
    D:/Work/CGALBuild/CGAL-4.14.3/Build/include \
    D:/Work/CGALBuild/CGAL-4.14.3/auxiliary/gmp/include

LIBS += -LD:/SoftWare/OpenCV/opencv/build/x64/vc15/lib/ -lopencv_world345 \
    -LD:/Work/CGALBuild/CGAL-4.14.3/Build/lib -lCGAL_Core-vc141-mt-4.14.3 -lCGAL-vc141-mt-4.14.3 \
    -LD:/Work/CGALBuild/CGAL-4.14.3/auxiliary/gmp/lib -llibgmp-10

#DESTDIR = $$PWD/../bin
