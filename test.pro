QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# Disable deprecated APIs
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    realsensewidget.cpp \
    robotmonitor.cpp

HEADERS += \
    mainwindow.h \
    realsensewidget.h \
    robotmonitor.h

FORMS += \
    mainwindow.ui

# 💡 OpenCV 설정
INCLUDEPATH += /usr/include/opencv4
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

# 💡 RealSense 설정
LIBS += -lrealsense2

# 💡 POSIX 공유 메모리 및 세마포어
LIBS += -lrt -lpthread

# API-DRFL 연결 설정
INCLUDEPATH += $$PWD/../API-DRFL/include
LIBS += -L$$PWD/../API-DRFL/library/Linux/64bits/amd64/22.04
LIBS += -lDRFL
LIBS += -lPocoFoundation
LIBS += -lPocoNet


# DRFL 라이브러리 링크 (실제 경로로 수정 필요)
exists($$DRFL_LIB) {
    LIBS += -L$$DRFL_LIB -lDRFL -lPOCO
    message("DRFL library path added: $$DRFL_LIB")
} else {
    warning("DRFL library path not found: $$DRFL_LIB")
    warning("Please set correct DRFL library path in .pro file")
}

# 💡 또는 직접 DRFL 경로를 지정하는 경우:
# INCLUDEPATH += /path/to/your/DRFL/include
# LIBS += -L/path/to/your/DRFL/lib -lDRFL -lPOCO

# Default rules for deployment
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
