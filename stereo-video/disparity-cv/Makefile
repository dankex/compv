OPENCV_HOME=/usr/local/Cellar/opencv/2.4.11

INCL=\
    -I$(OPENCV_HOME)/include/opencv\
    -I$(OPENCV_HOME)/include \
    -I../gsl\

LIBS=\
    -L$(OPENCV_HOME)/lib \
    -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -lopencv_calib3d

all: disparity

disparity: disparity.cpp
	g++ -o disparity disparity.cpp $(INCL) $(LIBS) 

