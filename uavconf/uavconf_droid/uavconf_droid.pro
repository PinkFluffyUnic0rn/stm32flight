QT += widgets
QT += core-private

requires(qtConfig(listview))

SOURCES   = ../api/uavconf.c \
            ../gui/main.cpp \
            ../gui/mainwidget.cpp
HEADERS   = \
    ../api/uavconf.h \
    ../gui/mainwidget.h

# install
target.path = $$[QT_INSTALL_EXAMPLES]/widgets/itemviews/uavconf_droid
INSTALLS += target
