TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main1.cpp\
            cable.cpp\


#           cable.cpp

HEADERS += cable.h


INCLUDEPATH +=/usr/include/boost\
     +=/usr/include/eigen3\
    +=/home/jiangxvv/programs/gsl2.4/include\

#LIBS +=/home/jiangxvv/programs/gsl2.4/lib/pkgconfig

RESOURCES += \
    resource.qrc

DISTFILES +=

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../programs/gsl2.4/lib/release/ -lgsl
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../programs/gsl2.4/lib/debug/ -lgsl
else:unix: LIBS += -L$$PWD/../../../programs/gsl2.4/lib/ -lgsl

INCLUDEPATH += $$PWD/../../../programs/gsl2.4/include
DEPENDPATH += $$PWD/../../../programs/gsl2.4/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../programs/gsl2.4/lib/release/ -lgslcblas
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../programs/gsl2.4/lib/debug/ -lgslcblas
else:unix: LIBS += -L$$PWD/../../../programs/gsl2.4/lib/ -lgslcblas

INCLUDEPATH += $$PWD/../../../programs/gsl2.4/include
DEPENDPATH += $$PWD/../../../programs/gsl2.4/include
