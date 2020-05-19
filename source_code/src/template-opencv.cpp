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
int slider_y = 259;
int slider_x_right = 508;

//std::vector<cv::Point2f> mcB;



double alpha;
double beta;

int bMinHue= 42, bMinSat=99, bMinVal= 44, bMaxHue=155, bMaxSat=200, bMaxVal=79;
int yMinHue= 18, yMinSat=101, yMinVal= 104, yMaxHue=53, yMaxSat=255, yMaxVal=255;

cv::Mat blueCones;
cv::Mat yellowCones;

using namespace cv;

Mat applyFilter(Mat img, int minHue, int minSat, int minVal, int maxHue, int maxSat, int maxVal);
std::vector<cv::Point2f>  findCoordinates(std::vector<std::vector<cv::Point> > contours);
Mat reduceNoise(Mat image);
Mat applyWarp(Mat image);
double calculateInverse(double bLength, double cLength);
double calculateAngle(double inverse);
static void makeTrackbar(Mat image, int WIDTH, int HEIGHT);
static void on_trackbar( int, void* );
bool checkSide(Mat image);
bool conesLeft;
double grndSteerAngle = 0;
int coneDecider=0;
int ind = 0;
double dis;
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
            std::int32_t time;
           
            auto onGroundSteeringRequest = [&gsr, &gsrMutex,&time, &grndSteerAngle](cluon::data::Envelope &&env){
                // The  envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                
               // std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
                //std::cout<< "At timeStamp= "<< env.sampleTimeStamp().seconds()<< " the groundSteering angle is: "<<  grndSteerAngle <<" original: " << gsr.groundSteering()<<std::endl;
                //std::cout<< env.sampleTimeStamp().seconds()<< " "<<  grndSteerAngle <<"; "<< gsr.groundSteering()<<std::endl;
                time= env.sampleTimeStamp().seconds();
            };
            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),onGroundSteeringRequest);
            
            dis; 
            opendlv::proxy::DistanceReading dr;
            std::mutex drMutex;

            auto onDistanceReadingRequest=[&dr, &drMutex, &dis](cluon::data::Envelope &&env){
                std::lock_guard<std::mutex> lck(drMutex);
                
                dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
                //std::cout << "distance from the file = " << dr.distance() << std::endl;
                dis= (dr.distance()/2)/29.1; 
                //dis = dr.distance();
                //std::cout << "actual distance = " << dis << " at this timeStamp: "<< env.sampleTimeStamp().seconds()<< std::endl;

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
                            
                makeTrackbar(img, WIDTH, HEIGHT);
                Mat warpedImgBlue;
                Mat warpedImgYellow;
                Mat warpedImgCombined;               

                //Both the blue and the yellow cones are givven a gaussian blur, dilated, and put through the canny method
                //Canny detects the edges of a given imag

                
                vector <Point2f> src_1[6000];
            
                warpedImgBlue = applyWarp(blueConesClose);
                warpedImgYellow = applyWarp(yellowConesClose);

                Mat cannyImage; 
                warpedImgCombined= warpedImgBlue + warpedImgYellow; 
                //Canny(warpedImgCombined, cannyImage, 127,255,3);

                //This combines the warped images for the blue and yellow cones.
                //warpedImgCombined = warpedImgBlue + warpedImgYellow; 
                RNG rng(12345);                
                Scalar color= Scalar(rng.uniform(0,225), rng.uniform(0,255), rng.uniform(0,255));            
                vector<vector<Point> > contoursB;
                vector<vector<Point> > contoursY;
                findContours(warpedImgBlue, contoursB, RETR_TREE,CHAIN_APPROX_SIMPLE); 
                findContours(warpedImgYellow, contoursY, RETR_TREE,CHAIN_APPROX_SIMPLE); 

                std::vector<cv::Point2f> mcB = findCoordinates(contoursB);
                std::vector<cv::Point2f> mcY = findCoordinates(contoursY);

                //Mat drawing= Mat::zeros(warpedImgCombined.size(), CV_8UC3);
                Mat drawing = warpedImgCombined.clone();
                Point lineStart = Point(320, 350);
                
                unsigned int len = 0;
                
                if(coneDecider==0){
                    checkSide(warpedImgBlue);
                }

                if(contoursB.size()>0 && contoursY.size() > 0){ 
                    if(mcB[0].y<350 && mcY[0].y<350) {   
                        double midpointX = (mcB[0].x + mcY[0].x)/2;
                        double midpointY = (mcB[0].y + mcY[0].y)/2;

                        Point2f midpoint = Point2f(midpointX, midpointY);

                        double oppLength = 320 - midpoint.x;
                        double adLength = 450 - midpoint.y;

                        double midpointRadian = calculateInverse(adLength, oppLength);
                        double midpointRadian2 = midpointRadian -(midpointRadian/2);
                        if(dis > 0.03){
                            grndSteerAngle = 0;
                        }else{ 
                            if(midpointRadian2< 0.3 && midpointRadian2 > -0.3){
                                grndSteerAngle = midpointRadian2;
                            }
                                
                        }
                        line(drawing, lineStart, midpoint, color, 5);
                        line(drawing, lineStart, Point(320, midpoint.y), Scalar(0,255,0), 5);
                        line(drawing, midpoint, Point(320, midpoint.y), Scalar(0,0,255), 5);
                    }
               } else if(contoursB.size()>0){
                        len = 0;   
                     if(mcB[len].y < 350){
                        len = contoursB.size()-1;
                        double cLength;
                        //circle(warpedImgCombined,mcB[len],4,color,-1,8,0);
                         if(conesLeft){
                              cLength = 320 - mcB[len].x;
                            }else{
                                cLength=mcB[len].x-320;
                            }
                        double bLength = 450 - mcB[len].y;
                        double radian {calculateInverse(bLength,cLength)};
                        double angle {calculateAngle(radian)};
                        double radian2 = radian - (radian/2);

                        if(radian2< 0.3 && radian2 > -0.3){
                            grndSteerAngle = radian - (radian/ 2);
                            }
                                              
                            line(drawing, lineStart, mcB[len], color, 5);
                            line(drawing, lineStart, Point(320, mcB[len].y), Scalar(0,255,0), 5);
                            line(drawing, mcB[len], Point(320, mcB[len].y), Scalar(0,0,255), 5);
                       }
                   }

                   Mat drawing2 = drawing + warpedImgCombined;

                // If you want to access the latest received ground steering, don't forget to lock the mutex:

                {
                    std::lock_guard<std::mutex> lck(gsrMutex);

                std::cout <<time<<"; "<<grndSteerAngle<< "; " << gsr.groundSteering() << std::endl;
                    
                }
                coneDecider++;
                
                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    
                    //cv::imshow("with rect", drawing);
                    cv::imshow("cones", drawing);
                    //cv::imshow("yellow cones", yellowCones);
                    //cv::imshow("blue cones", blueCones);
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
    Mat gBlurredImg;
    Mat dilatedImg;
    Mat cannyImg;

    //cv::Mat Kernel = cv::Mat(cv::Size(5,5),CV_8UC1,cv::Scalar(255));
    //cv::morphologyEx(image, imgOpen,cv::MORPH_OPEN,Kernel);
    //cv::morphologyEx(imgOpen, imgClose,cv::MORPH_CLOSE,Kernel); 
    
    GaussianBlur(image,gBlurredImg,Size(5,5),0);
    dilate(gBlurredImg, dilatedImg, Mat(), Point(-1, -1), 2, 1, 1); 
    Canny(gBlurredImg, cannyImg, 127,255,3);



    return cannyImg;

}

double calculateInverse(double bLength, double cLength){
    if(bLength && cLength != 0){
    double inverse= atan(cLength/bLength);
    return inverse;
    }
    return 0;
}

double calculateAngle(double inverse){
    double angle=(inverse*180)/3.1415;
    return angle;
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
using namespace cv;
using namespace std;

std::vector<cv::Point2f> findCoordinates(std::vector<std::vector<cv::Point> > contours){
    std::vector<std::vector<cv::Point> > contour_polyB(contours.size() );
    std::vector<Rect> boundRectB( contours.size() );
    std::vector<Moments> muB (contours.size());
    std::vector<cv::Point2f> mc (contours.size());

    for(size_t i=0; i<contours.size();i++){
            muB[i]= moments(contours[i], false);
            approxPolyDP(contours[i], contour_polyB[i], 3, true); 
            boundRectB[i]= boundingRect(contour_polyB[i]);
    }

    //mcB.resize(contours.size());
    for(size_t i=0; i< contours.size();i++){
            mc[i]= Point2f(muB[i].m10/muB[i].m00, muB[i].m01/muB[i].m00);
     }

     return mc;     
}

static void makeTrackbar(Mat image, int WIDTH, int HEIGHT){
        slider_dst = image.clone();
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
}

static void on_trackbar( int, void* ){
   cv::Point left = cv::Point(slider_x_left, slider_y);
   cv::Point right = cv::Point(slider_x_right, slider_y);
   cv::Scalar color= cv::Scalar(255,0,0);
   cv::circle(slider_dst, left,4,color,-1,8,0); 
   cv::circle(slider_dst, right,4,color,-1,8,0); 
   cv::imshow( "Linear Blend", slider_dst);
}
bool checkSide(Mat image){
    
    int count;
    int count2;
    cv::Mat left;
    cv::Mat right;
    cv::Rect leftPart(0,0,320,480);
    cv::Rect rightPart(320,0, 320,480);
    left=image(leftPart);
    right=image(rightPart);
    count= cv::countNonZero(left);
    count2= cv::countNonZero(right);
    if(count<count2){
        //std::cout<<"cone are right"<<std::endl;
        conesLeft=false;
    }else{
        conesLeft=true;
        //std::cout<<"cones are left"<<std::endl;
    }
    
    return conesLeft;

}

