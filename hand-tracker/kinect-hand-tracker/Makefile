OSTYPE := $(shell uname -s)

SRC_FILES = HandTracker.cpp PointDrawer.cpp signal_catch.cpp graphcut.cpp

# CFG = Release
CFG = Debug

ifeq ("$(CFG)", "Debug")
    DEFINES += DEBUG MALLOC_CHECK_=3
endif

BIN_DIR = Bin

INC_DIRS += /usr/include/ni /usr/include/nite

EXE_NAME = HandTracker

ifeq ("$(OSTYPE)","Darwin")
	LDFLAGS += -framework OpenGL -framework GLUT
else
	USED_LIBS += glut GL
endif

USED_LIBS += OpenNI XnVNite_1_3_1

include /home/danke/p/compv/openni/OpenNI/Platform/Linux/Build/Common/CommonCppMakefile
