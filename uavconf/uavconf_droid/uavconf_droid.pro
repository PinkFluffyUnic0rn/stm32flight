QT += widgets
QT += core-private

requires(qtConfig(listview))

SOURCES   = ../api/uavconf.c \
            ../gui/main.cpp \
            ../gui/mainwidget.cpp
HEADERS   = \
    ../api/uavconf.h \
    ../gui/mainwidget.h \
    uavconf.h

# install
target.path = $$[QT_INSTALL_EXAMPLES]/widgets/itemviews/uavconf_droid
INSTALLS += target

DISTFILES += \
    android/AndroidManifest.xml \
    android/build.gradle \
    android/gradle.properties \
    android/gradle/wrapper/gradle-wrapper.jar \
    android/gradle/wrapper/gradle-wrapper.properties \
    android/gradlew \
    android/gradlew.bat \
    android/res/values/libs.xml \
    android/res/xml/qtprovider_paths.xml

