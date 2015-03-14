#-------------------------------------------------
#
# Project created by QtCreator 2014-12-01T21:09:37
#
#-------------------------------------------------

QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets serialport opengl printsupport

TARGET = EvvGCPConf32
TEMPLATE = app


SOURCES += \
    main.cpp\
    mainwindow.cpp \
    glwidget.cpp \
    serialthread.cpp \
    crc32.cpp \
    3rdparty/qcustomplot.cpp

HEADERS += \
    mainwindow.h \
    glwidget.h \
    serialthread.h \
    telemetry.h \
    crc32.h \
    3rdparty/qcustomplot.h

FORMS += \
    mainwindow.ui

RESOURCES += \
    textures.qrc
