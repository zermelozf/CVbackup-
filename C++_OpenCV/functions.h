#include <iostream>
#include <vector>


#include <iomanip>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

#include "CameraCalibrator.h"

// Reduce the color space of the image to div bits
Mat colorReduce(const Mat &image, int div);

// Transform the image to a binary one where only the square should be on
Mat toBinary(const Mat &image);

// Find the position of the square and draw it
void detectSquaresAndDraw(const Mat &frame, Mat &augmentedFrame, vector<vector<Point> > &squarePos);

// Play the video stream from the camera or the input file
int process(VideoCapture& capture, CameraCalibrator cp, bool augmented);

// plop
int plop(const Mat &cameraMatrix, const vector<Point> edges, Mat &image, Mat &rvec, Mat &tvec);