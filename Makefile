ARM_ROOT = /home/ubuntu/arm-control
DARKNET_ROOT = /home/ubuntu/darknet

ACTION_RETRY = y

CC = g++
CFLAGS = -Wall -std=c++11 -DDEBUG  -D__CHAMFER_DEBUG_MODE___ -DTEST_TEMPLATE -D__MATCH_DEBUG_MODE___
CFLAGS += -D__CHAMFER_INFO_REPORT___

ifeq ($(ACTION_RETRY), y)
CFLAGS += -DRETRY
endif

SOURCES = main.cpp $(ARM_ROOT)/ros/src/ros_wrapper.cpp

ifeq ($(ACTION_RETRY), y)
SOURCES += action.cpp
endif

EXECUTABLE = ../pick-and-place

INCLUDE = -I$(ARM_ROOT)/arm/inc -I$(ARM_ROOT)/ros/inc -I$(DARKNET_ROOT)/ChamferMatchingTest/ChamferMatching `pkg-config --cflags opencv`
LDFLAGS = -L$(DARKNET_ROOT) -L$(ARM_ROOT)/arm -L$(DARKNET_ROOT)/ChamferMatchingTest/ChamferMatching

INCLUDE += -I/opt/ros/indigo/include -I/home/ubuntu/librealsense-armhf/examples/include -I$(ARM_ROOT)/ros/inc
LDFLAGS += -L/opt/ros/indigo/lib -L/usr/local/cuda-6.5/lib -L/usr/lib
LIBRARY += -lopencv_core -lopencv_imgproc -lopencv_flann  -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_objdetect -lopencv_photo -lopencv_features2d -lopencv_calib3d -lopencv_stitching  -Iinclude -Llib -lrealsense -lroscpp -lrosconsole -lrostime -lroscpp_serialization  -lm `pkg-config --cflags --libs glfw3 glu gl` 

LIBRARY += -ldarknet -larm -lSource

all:
	$(CC) $(CFLAGS) $(INCLUDE) $(LDFLAGS) $(SOURCES) -o $(EXECUTABLE) $(LIBRARY)

test_plate:
	$(CC) $(CFLAGS) $(INCLUDE) $(LDFLAGS) test_plate.cpp test.cpp -o ../test_plate $(LIBRARY)

test_plate_contour:
	$(CC) $(CFLAGS) $(INCLUDE) $(LDFLAGS) test_plate_contour.cpp -o ../test_plate_contour $(LIBRARY)

