ACLOCAL_AMFLAGS = -I m4

## build libcamera ##
lib_LTLIBRARIES = libcamera.la

include_HEADERS = inc/camera.h
include_HEADERS += inc/camera_parameters.h

libcamera_la_CXXFLAGS  = -fPIC

libcamera_la_CXXFLAGS += -I src
libcamera_la_CXXFLAGS += -I inc
libcamera_la_CXXFLAGS += -I ../QCamera2/HAL
libcamera_la_CXXFLAGS += -I ../QCamera2/util
libcamera_la_CXXFLAGS += -I ../QCamera2/stack/common
libcamera_la_CXXFLAGS += -I ../mm-image-codec/qomx_core
libcamera_la_CXXFLAGS += -I ../mm-image-codec/qexif


libcamera_la_SOURCES  = src/qcamera2.cpp
libcamera_la_SOURCES += src/camera_memory.cpp
libcamera_la_SOURCES += src/camera_parameters.cpp

libcamera_la_LDFLAGS = -shared
libcamera_la_LIBADD = -ldl -lrt -lcamparams

## build camera-test ##

bin_PROGRAMS = camera-test

camera_test_CXXFLAGS  = -I inc
camera_test_CXXFLAGS += -I src
camera_test_CXXFLAGS += -I test
camera_test_CXXFLAGS += -I ../QCamera2/HAL

camera_test_SOURCES = test/camera_test.cpp

camera_test_LDADD = libcamera.la -lturbojpeg -lpthread
