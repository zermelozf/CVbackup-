#include "functions.h"

Mat colorReduce(const Mat &image, int div=64) {
    
    int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
    // mask used to round the pixel value
    uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
    
    Mat_<Vec3b>::const_iterator it= image.begin<Vec3b>();
    Mat_<Vec3b>::const_iterator itend= image.end<Vec3b>();
    
    // Set output image (always 1-channel)
    Mat result(image.rows,image.cols,image.type());
    Mat_<Vec3b>::iterator itr= result.begin<Vec3b>();
    
    for ( ; it!= itend; ++it, ++itr) {
        
        (*itr)[0]= ((*it)[0]&mask) + div/2;
        (*itr)[1]= ((*it)[1]&mask) + div/2;
        (*itr)[2]= ((*it)[2]&mask) + div/2;
    }
    
    return result;
}

Mat toBinary(const Mat &image) {
    Mat b;
    image.copyTo(b);
    for(int i=0; i<image.rows;i++) {
        for(int j=0;j<image.cols;j++) {
            if(image.at<cv::Vec3b>(i,j)[0] > 160 && image.at<cv::Vec3b>(i,j)[1] > 160 && image.at<cv::Vec3b>(i,j)[2] > 160) {
                b.at<cv::Vec3b>(i,j)[0] = 255;
                b.at<cv::Vec3b>(i,j)[1] = 255;
                b.at<cv::Vec3b>(i,j)[2] = 255;
            }
            else { 
                b.at<cv::Vec3b>(i,j)[0] = 0;
                b.at<cv::Vec3b>(i,j)[1] = 0;
                b.at<cv::Vec3b>(i,j)[2] = 0; 
            }
        }
    }
    return b;
}

void detectSquaresAndDraw(const Mat &frame, Mat &augmentedFrame, vector<Point> &squarePos)
{
	// Read reference image
	//Mat image= cv::imread("/Users/bcolas/Documents/Computer Vision/C++_OpenCV/data/photo.JPG");
    Mat image;
    frame.copyTo(image);
    //image = colorReduce(image, 32);
    //namedWindow("Image");
	//imshow("Image",image);
    
    // Convert to GrayScale image
    Mat binIm = toBinary(image);
    //adaptiveThreshold(image, binIm, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 2);
    Mat gray;
    cvtColor(binIm,gray,CV_RGB2GRAY);
    //namedWindow("GrayScale");
	//imshow("GrayScale",gray);
    //return gray;
    
    // Detect Contours in image and find squares
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Point> approx;
    vector<vector<Point> > squares;
    Mat filledCont = Mat::zeros(gray.rows, gray.cols, CV_8UC3);
    
    // Approximate Contours and select squares
    squares.clear();
    findContours(gray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for( int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
    {
        Scalar color( rand()&255, rand()&255, rand()&255 );
        approxPolyDP(Mat(contours[idx]), approx, arcLength(Mat(contours[idx]), true)*0.05, true);
        drawContours( filledCont, contours, idx, color, CV_FILLED, 8, hierarchy );
        if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))) {
            squares.push_back(approx);
        }
        
    }
    //namedWindow("Filled Contours");
	//imshow("Filled Contours",filledCont);
    
    // Draw Squares on the image
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(255,0,0), 2, CV_AA);
        
        // Colors to determin the order of the points in the vector
        circle(image, squares[i][0], 5,Scalar(255,255,255), 2);
        circle(image, squares[i][1], 5,Scalar(0,0,255), 2);
        circle(image, squares[i][2], 5,Scalar(0,255,0), 2);
        circle(image, squares[i][3], 5,Scalar(255,0,0), 2);
    }
    //namedWindow("Squares");
    //imshow("Squares", image);
    
    augmentedFrame = image;
    //augmentedFrame = filledCont;
    //augmentedFrame = binIm;
    squarePos = approx;
    // Wait before exit
	//waitKey();
}

int plop(const Mat &cameraMatrix, const vector<Point> edges, Mat &image, Mat &rvec, Mat &tvec) {
    
    // 3D Scene Points 
    Size boardSize(9,6);
    vector<Point3f> objectCorners;
    for (int i=0; i<boardSize.height; i++) {
		for (int j=0; j<boardSize.width; j++) {
			objectCorners.push_back(Point3f(i, j, 0.0f));
		}
    }
    
    // Image Points
    Mat gray;
    Mat dist;
    cvtColor(image,gray,CV_RGB2GRAY);
    vector<Point2f> imageCorners;
    findChessboardCorners(gray, boardSize, imageCorners);
    if (imageCorners.size() == boardSize.area()) {
        cornerSubPix(gray, imageCorners, Size(5,5), Size(-1,-1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));
        solvePnP(objectCorners, imageCorners, cameraMatrix, dist, rvec, tvec, true);
        stringstream d;
        d << abs(tvec.at<double>(0,2))*2.2 << "cm";
        putText(image, d.str(), Point(450,30), FONT_HERSHEY_PLAIN, 2, Scalar(255,255,255),2);
        //putText(image, d.str(), Point(100,100), FONT_HERSHEY_PLAIN, 5, Scalar(255,255,255),2);

        cout << "tvec" << rvec.total() << endl;
        cout << tvec.at<double>(0,0) << " " << tvec.at<double>(0,1) << " " << tvec.at<double>(0,2) << endl;
        
        
        /*
         cout << "rvec" << rvec.total()<< endl;
         cout << rvec.at<double>(0,0) << " " << rvec.at<double>(0,1) << " " << rvec.at<double>(0,2) << endl;
         cout << rvec.at<double>(1,0) << " " << rvec.at<double>(1,1) << " " << rvec.at<double>(1,2) << endl;
         cout << rvec.at<double>(2,0) << " " << rvec.at<double>(2,1) << " " << rvec.at<double>(2,2) << endl;*/
        
        
        vector<Point3f> pobjV;
        float c = 3.0;
        Point3f t(0, 0, 0);
        pobjV.push_back(Point3f(0,0,0)+t);
        pobjV.push_back(Point3f(0,0,c)+t);
        pobjV.push_back(Point3f(0,c,0)+t);
        pobjV.push_back(Point3f(0,c,c)+t);
        pobjV.push_back(Point3f(c,0,0.0f)+t);
        pobjV.push_back(Point3f(c,0,c)+t);
        pobjV.push_back(Point3f(c,c,0.0f)+t);
        pobjV.push_back(Point3f(c,c,c)+t);
        vector<Point2f> pimV; 

        /*
        Mat distCoeffs = (Mat_<double>(4,1) << 0, 0, 0, 0);
        Mat cameraMatrix = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
        Mat rvec = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        Mat tvec = (Mat_<double>(3,1) << 0, 0, 0);
         */
        Mat distCoeffs = (Mat_<double>(4,1) << 0, 0, 0, 0);
        
        //projectPoints(pobjV, rvec, tvec, cameraMatrix, distCoeffs, pimV);
        projectPoints(pobjV, rvec, tvec, cameraMatrix, distCoeffs, pimV);

        cout << "center: " << pimV[0].x << pimV[0].y << endl;
        //Point center = pimV[0];
        line(image, pimV[0], pimV[1], Scalar(0,255,0), 2);
        line(image, pimV[2], pimV[3], Scalar(0,255,0), 2);
        line(image, pimV[0], pimV[2], Scalar(0,255,0), 2);
        line(image, pimV[1], pimV[3], Scalar(0,255,0), 2);
        line(image, pimV[4], pimV[5], Scalar(0,255,0), 2);
        line(image, pimV[6], pimV[7], Scalar(0,255,0), 2);
        line(image, pimV[4], pimV[6], Scalar(0,255,0), 2);
        line(image, pimV[5], pimV[7], Scalar(0,255,0), 2);
        line(image, pimV[0], pimV[4], Scalar(0,255,0), 2);
        line(image, pimV[2], pimV[6], Scalar(0,255,0), 2);
        line(image, pimV[1], pimV[5], Scalar(0,255,0), 2);
        line(image, pimV[3], pimV[7], Scalar(0,255,0), 2);

    }
	return 1;
}


int process(VideoCapture& capture, CameraCalibrator cp, bool augmented) {
    int n = 0;
    char filename[200];
    string window_name = "video | q or esc to quit";
    cout << "press space to save a picture. q or esc to quit" << endl;
    

    
    VideoWriter vWriter;
    string oVideoFile = "/Users//Documents/Computer Vision/C++_OpenCV/data/recorded.avi";
    namedWindow(window_name, CV_WINDOW_KEEPRATIO); //resizable window;
    Mat frame;
    Mat augmentedFrame;
    Mat rvec ,tvec; 
    Mat camMat;
    for (;;) {
        capture >> frame;
        if (frame.empty())
            continue;
        // Process the frame and draw square
        // Here the interesting part takes place
        if (augmented) {
            vector<Point> squareEdges;
            detectSquaresAndDraw(frame, augmentedFrame, squareEdges);
            camMat = cp.getCameraMatrix();
            plop(camMat, squareEdges, augmentedFrame, rvec, tvec);
            imshow(window_name, augmentedFrame);
            
        // For Writing
            
        }
        else {
            //putText(frame, "plop", Point(50,50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255,255,255), 5);
            imshow(window_name, frame);
        }
        char key = (char)waitKey(5); //delay N millis, usually long enough to display and capture input
        switch (key) {
            case 'q':
            case 'Q':
            case 27: //escape key
                return 0;
            case ' ': //Save an image
                if (augmented) {
                    sprintf(filename,"/Users/bcolas/Documents/Computer Vision/C++_OpenCV/data/augFrame%.3d.jpg",n++);
                    imwrite(filename,augmentedFrame);
                }
                else {
                    sprintf(filename,"/Users/bcolas/Documents/Computer Vision/C++_OpenCV/data/CalibrateCam%.3d.jpg",n++);
                    imwrite(filename,frame);
                }
                cout << "Saved " << filename << endl;
                break;
            default:
                break;
        }
    }
    return 0;
}