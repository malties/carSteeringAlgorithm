/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications
#include "opendlv-standard-message-set.hpp"
//matplot python library wrapped for c++

 
// Include the GUI and image processing header files from OpenCV
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <vector>

const int alpha_slider_max = 640;
int slider_x_left = 92;
int slider_y = 276;
int slider_x_right = 508;


double alpha;
double beta;

int bMinHue= 42, bMinSat=99, bMinVal= 44, bMaxHue=155, bMaxSat=200, bMaxVal=79;
int yMinHue= 18, yMinSat=101, yMinVal= 104, yMaxHue=53, yMaxSat=255, yMaxVal=255;

cv::Mat blueCones;
cv::Mat yellowCones;

using namespace cv;

static void on_trackbar( int, void* );
Mat applyFilter(Mat img, int minHue, int minSat, int minVal, int maxHue, int maxSat, int maxVal);
Mat reduceNoise(Mat image);
Mat applyWarp(Mat image);

cv::Mat img;
cv::Mat slider_dst;
 
int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
 
        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;
 
            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
 
            opendlv::proxy::GroundSteeringRequest gsr;
            
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                
               // std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
                std::cout<< "At timeStamp= "<< env.sampleTimeStamp().seconds()<< "the groundSteering angle is: "<<  gsr.groundSteering()<<std::endl;
            };
            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),onGroundSteeringRequest);
            
            double dis; 
            opendlv::proxy::DistanceReading dr;
            std::mutex drMutex;

            auto onDistanceReadingRequest=[&dr, &drMutex, &dis](cluon::data::Envelope &&env){
                std::lock_guard<std::mutex> lck(drMutex);
                
                dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
                //std::cout << "distance from the file = " << dr.distance() << std::endl;
                dis= (dr.distance()/2)/29.1;
                //std::cout << "actual distance = " << dis << "at this timeStamp: "<< env.sampleTimeStamp().seconds()<< std::endl;

            };

            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(),onDistanceReadingRequest);
            
            
            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                // OpenCV data structure to hold an image.
                //cv::Mat img;
 
                // Wait for a notification of a new frame.
                sharedMemory->wait();
 
                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                
                
                /*
                cluon::data::TimeStamp tid= cluon::time::now();
                int64_t na= cluon::time::toMicroseconds(tid);
                std::cout << "the timeStamps"<< na<< endl;
                */
                sharedMemory->unlock();
 
                // TODO: Do something with the frame.
                // Example: Draw a red rectangle and display image.
                //cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));
 
                using namespace std;
                //using namespace cv;

                cv::Mat hsv;

                cv::Mat blueConesOpen;
                cv::Mat blueConesClose;

                cv:: Mat yellowConesOpen;
                cv:: Mat yellowConesClose;

                Mat imgCopyBlue = img.clone();
                Mat imgCopyYellow = img.clone();
                
                blueCones= applyFilter(imgCopyBlue, 42, 99, 44, 155, 200, 79);
                yellowCones= applyFilter(imgCopyYellow, yMinHue, yMinSat, yMinVal, yMaxHue, yMaxSat, yMaxVal);
               

                //Opening and closing are used for getting rid of noise

                blueConesClose = reduceNoise(blueCones);
                yellowConesClose = reduceNoise(yellowCones);
              

               //The code below is the code that is currently being used             

                //The following code between line 171 and 187 is used for making trackbars that adjust the x and y coordinates
                //for the top two warp points. It's used to make finding the points on the image easier
                slider_dst = img.clone();
                namedWindow("Linear Blend", WINDOW_AUTOSIZE);  //This is the window that the track bar will be displayed in
                char TrackbarName[50];  //Each trackbar has a name
                char TrackbarName2[50];
                char TrackbarName3[50];
    
                sprintf( TrackbarName, "Point 1 x: %d", WIDTH );  //This sets the length for each trackbar as well as the text beside it
                sprintf( TrackbarName2, "Point 2 x: %d", WIDTH );
                sprintf( TrackbarName3, "Point y: %d", HEIGHT );
                //This creates the trackbar.  Includes its name, the name of the window it will appear in, the starting value of its slider,
                //the maximum value of its slider, and the method that will be called when it is moved.
                createTrackbar( TrackbarName, "Linear Blend", &slider_x_left, alpha_slider_max, on_trackbar );
                createTrackbar( TrackbarName2, "Linear Blend", &slider_x_right, alpha_slider_max, on_trackbar );
                createTrackbar( TrackbarName3, "Linear Blend", &slider_y, alpha_slider_max, on_trackbar );
                //these are the methods that are called when the trackbar is moved.
                on_trackbar( slider_x_left, 0 );
                on_trackbar( slider_y, 0 );
                on_trackbar( slider_x_right, 0 );
                //These are the output and input values for the various imaging filtering methods
                
                Mat warpedImgBlue;
                Mat warpedImgYellow;
                Mat warpedImgCombined;

                //Both the blue and the yellow cones are givven a gaussian blur, dilated, and put through the canny method
                //Canny detects the edges of a given imag

                //might be unnecessary
                vector <Point2f> src_1[6000];
                //These are the points from the original image that will be stretched out in the perspective warp.  
                //pts1 is the array of points.  
                /*vector <Point2f> pts1;
                pts1.push_back(Point2f(slider_x_left, slider_y));  //The x and y coordinates of the top two points can be adjusted with the
                pts1.push_back (Point2f(slider_x_right, slider_y)); //track bar
                pts1.push_back(Point2f(0, 386));
                pts1.push_back (Point2f(632, 386));*/
                //These are the corners of the new image that the section of the image within the previous points will be sretched into
                //These points basically determine the width and height of the new image which will be the same as the original image
                /*vector <Point2f> pts2;
                pts2.push_back(Point2f(0,0));
                pts2.push_back(Point2f(WIDTH,0));
                pts2.push_back(Point2f(0,HEIGHT));
                pts2.push_back(Point2f(WIDTH,HEIGHT));*/
                //The matrix in the points on the original image that are to be stretched out and the new points thew will be 
                //stretched out to
                
                warpedImgBlue = applyWarp(blueConesClose);
                warpedImgYellow = applyWarp(yellowConesClose);

                Mat cannyImage; 
                warpedImgCombined= warpedImgBlue + warpedImgYellow; 
                Canny(warpedImgCombined, cannyImage, 127,255,3);

                //This combines the warped images for the blue and yellow cones.
                //warpedImgCombined = warpedImgBlue + warpedImgYellow; 


                RNG rng(12345);
                
                Scalar color= Scalar(rng.uniform(0,225), rng.uniform(0,255), rng.uniform(0,255));
               
            
                vector<vector<Point> > contoursB;
                vector<vector<Point> > contoursY;

                findContours(warpedImgBlue, contoursB,RETR_TREE,CHAIN_APPROX_SIMPLE); 
                //findContours(warpedImgYellow, contoursY,RETR_TREE,CHAIN_APPROX_SIMPLE);

                vector<vector<Point> > contour_polyB(contoursB.size() );
                //vector<vector<Point> > contour_polyY(contoursY.size() );
                vector<Rect> boundRectB( contoursB.size() );
                //vector<Rect> boundRectY( contoursY.size() );
                vector<Moments> muB (contoursB.size());
               // vector<Moments> muY (contoursY.size());

                //the following is necessary to find the edges of a square that can contain the cones.
                 for(size_t i=0; i<contoursB.size();i++){
                    muB[i]= moments(contoursB[i], false);
                    approxPolyDP(contoursB[i], contour_polyB[i], 3, true); 
                    boundRectB[i]= boundingRect(contour_polyB[i]);
                 }

                 //following is code for yellow 
                 /*
                  for(size_t i=0; i<contoursY.size();i++){
                    muY[i]= moments(contoursY[i], false);
                    approxPolyDP(contoursY[i], contour_polyY[i], 3, true); 
                    boundRectY[i]= boundingRect(contour_polyY[i]);
                 }
                 */

                Mat drawing= Mat::zeros(cannyImage.size(), CV_8UC3);
                vector<Point2f> mcB (contoursB.size());
                
                //vector<Point2f> mcY (contoursY.size());

                //The following fills out mcB with cones' coordinates 
                for(int unsigned i=0; i< contoursB.size();i++){
                    mcB[i]= Point2f(muB[i].m10/muB[i].m00, muB[i].m01/muB[i].m00);
                }
                /*
                for(int unsigned i=0; i< contoursY.size();i++){
                    mcY[i]= Point2f(muY[i].m10/muY[i].m00, muY[i].m01/muY[i].m00);
                }
                */

                double aLength[1000];
                double bLength;
                double cLength;
                double angle;
                float value; 
                double inverse;

                Point lineStart = Point(320, 450);
                for(int unsigned i =0; i<contoursB.size(); i++){
                    drawContours(drawing, contour_polyB, (int)i, color);
                    rectangle(drawing,boundRectB[i].tl(), boundRectB[i].br(), color,2);
                    circle(drawing,mcB[i],4,color,-1,8,0);
                   /* if(i>0) {
                        line(drawing, mcB[i-1], mcB[i], color,5 );
                    } */  

                    line(drawing, lineStart, mcB[i], color, 5);
                    line(drawing, lineStart, Point(320, mcB[i].y), Scalar(0,255,0), 5);
                    line(drawing, mcB[i], Point(320, mcB[i].y), Scalar(0,0,255), 5);

                    bLength = 450 - mcB[i].y;
                    cLength =  320 - mcB[i].x;
                    

                   // error handling  if(length==-nan)
                   // aLength = sqrt(pow(bLength,2) + pow(cLength,2));
                    
                    inverse= atan(cLength/bLength);
                   
                    //cout<<endl;
                    //cout<<endl;
                    
            
                    cout<<"the inverse "<<inverse<<endl;
                    angle=inverse*180/3.1415;

                    cout <<"adjacent is "<<bLength<<"the opposit"<<cLength<<"the angle in degress"<< angle<< "in radian"<< inverse<<endl;
                }
                double o=4;
                double a=7;
                double g;
                g= atan(o/a);

               
                double deg;
                deg= g*180/3.1415;
                cout<< "test in degrees "<<deg<< "in radian"<< g<<endl;
        
                
                /*
                for(int unsigned i =0; i<contoursY.size(); i++){
                    drawContours(drawing, contour_polyY, (int)i, color);
                    rectangle(drawing,boundRectY[i].tl(), boundRectY[i].br(), color,2);
                    circle(drawing,mcY[i],4,color,-1,8,0);
                    if(i>0) {
                        line(drawing, mcY[i-1], mcY[i], color,5 );
                    }
                    line(drawing, lineStart, mcY[i], Scalar(0,255,0), 5);                    
                }
                */

                // If you want to access the latest received ground steering, don't forget to lock the mutex:

                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }
                
                
                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    
                    //cv::imshow("with rect", drawing);
                    cv::imshow("cones", warpedImgBlue);
                    cv::imshow("yellow cones", yellowCones);
                    cv::imshow("blue cones", blueCones);
                    //cv::imshow("with g blurr", gBlurredImg);
                    //cv::imshow("with dilation", dilatedImg);
                    //cv::imshow("with dilation and canny", cannyDilateYellow);

                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

static void on_trackbar( int, void* )
{
   cv::Point left = cv::Point(slider_x_left, slider_y);
   cv::Point right = cv::Point(slider_x_right, slider_y);
   cv::Scalar color= cv::Scalar(255,0,0);
   cv::circle(slider_dst, left,4,color,-1,8,0); 
   cv::circle(slider_dst, right,4,color,-1,8,0); 
   cv::imshow( "Linear Blend", slider_dst);
}
Mat applyFilter(Mat image, int minHue, int minSat, int minVal, int maxHue, int maxSat, int maxVal){
    Mat hsv;
    Mat filteredCones;
    cvtColor(image,hsv,COLOR_BGR2HSV); 
    inRange(hsv, Scalar(minHue,minSat,minVal),Scalar(maxHue,maxSat,maxVal), filteredCones);
    
    return filteredCones;
}

Mat reduceNoise(Mat image){
    Mat imgOpen;
    Mat imgClose;
    cv::Mat Kernel = cv::Mat(cv::Size(5,5),CV_8UC1,cv::Scalar(255));
    cv::morphologyEx(image, imgOpen,cv::MORPH_OPEN,Kernel);
    cv::morphologyEx(imgOpen, imgClose,cv::MORPH_CLOSE,Kernel); 

    return imgClose;
}

Mat applyWarp(Mat image){
    Mat warpedImg;
    Mat matrix; 
    std::vector <Point2f> pts1;
    pts1.push_back(Point2f(slider_x_left, slider_y));  //The x and y coordinates of the top two points can be adjusted with the
    pts1.push_back (Point2f(slider_x_right, slider_y)); //track bar
    pts1.push_back(Point2f(0, 386));
    pts1.push_back (Point2f(632, 386));

    std::vector <Point2f> pts2;
    pts2.push_back(Point2f(0,0));
    pts2.push_back(Point2f(640,0));
    pts2.push_back(Point2f(0,480));
    pts2.push_back(Point2f(640,480));

    matrix = getPerspectiveTransform(pts1,pts2);
    warpPerspective(image, warpedImg, matrix, img.size());
    return warpedImg;
}

