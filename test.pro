# -------------------------------------------------
# Qt 모듈 설정 (필수)
# -------------------------------------------------
QT       += core gui widgets openglwidgets
# -------------------------------------------------
# C++ 표준 및 기본 설정
# -------------------------------------------------
CONFIG += c++11
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

# -------------------------------------------------
# 소스 및 헤더 파일
# -------------------------------------------------
SOURCES += \
    main.cpp \
    mainwindow.cpp \
    realsensewidget.cpp \
    robotcontroller.cpp \
    robotmonitor.cpp \
    xyplotwidget.cpp

HEADERS += \
    circlefitter.h \
    handleplotwidget.h \
    mainwindow.h \
    realsensewidget.h \
    robotcontroller.h \
    robotmonitor.h \
    xyplotwidget.h

FORMS += \
    mainwindow.ui

# -------------------------------------------------
# 외부 라이브러리 설정
# -------------------------------------------------

# OpenCV
INCLUDEPATH += /usr/include/opencv4
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

# RealSense
LIBS += -lrealsense2

# ✨ [수정] OpenGL 유틸리티 라이브러리(GLU) 추가
LIBS += -lGLU

# POSIX 공유 메모리 및 세마포어
LIBS += -lrt -lpthread

# API-DRFL
INCLUDEPATH += $$PWD/../API-DRFL/include
LIBS += -L$$PWD/../API-DRFL/library/Linux/64bits/amd64/22.04
LIBS += -lDRFL -lPocoFoundation -lPocoNet
INCLUDEPATH += /usr/include/eigen3
# -------------------------------------------------
# 배포 설정
# -------------------------------------------------
unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
