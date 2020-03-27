
SRCS = calibration_camera.cpp
DLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_calib3d

all:
	g++ -o calibration $(SRCS)  $(DLIBS) 
clean:
	rm calibration -rf
	rm result/*_d.JPG -rf
