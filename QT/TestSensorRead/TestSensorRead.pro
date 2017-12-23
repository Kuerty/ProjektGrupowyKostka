#-------------------------------------------------
#
# Project created by QtCreator 2016-10-21T18:20:05
#
#-------------------------------------------------

QT += widgets core gui serialport network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TestSensorRead
TEMPLATE = app


SOURCES += main.cpp\
        dialog.cpp

HEADERS  += dialog.h

FORMS    += dialog.ui
