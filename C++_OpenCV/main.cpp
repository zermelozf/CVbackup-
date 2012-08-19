
#include "functions.h"

int main(int ac, char** av) {
    
    if (ac != 2) {
        return 1;
    }
    string arg = av[1];
    VideoCapture capture(arg); //try to open string, this will attempt to open it as a video file
    if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
        capture.open(atoi(arg.c_str()));
    if (!capture.isOpened()) {
        cerr << "Failed to open a video device or video file!\n" << endl;
        return 1;
    }
    CameraCalibrator camParams;
    process(capture, camParams, false);
    camParams = initCalibration();
    return process(capture, camParams, true);
}
