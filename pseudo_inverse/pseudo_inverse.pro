TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp\
           variable_thruster_allocation.cpp\
           basic_controller.cpp\
            feedback_linear_controller.cpp

HEADERS+=variable_thruster_allocation.h\
        feedback_linear_controller.h\
          basic_controller.h\




#INCLUDEPATH += $$PWD/gnc_controller
#DEPENDPATH += $$PWD/
LIS+=./libgnc_controller.so

INCLUDEPATH +=/usr/include/boost\
     +=/usr/include/eigen3\


unix:!macx: LIBS += -L$$PWD/./ -lgnc_controller

INCLUDEPATH += $$PWD/gnc_controller
DEPENDPATH += $$PWD/gnc_controller
