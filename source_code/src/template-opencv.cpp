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
 
// Include the GUI and image processing header files from OpenCV
#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


const int alpha_slider_max = 640;
int slider_x_left = 92;
int slider_y = 276;
int slider_x_right = 508;


double alpha;
double beta;

static void on_trackbar( int, void* );

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
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };
 
            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
 
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
                sharedMemory->unlock();
 
                // TODO: Do something with the frame.
                // Example: Draw a red rectangle and display image.
                //cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));
 
                using namespace std;
                using namespace cv;
                cv::Mat hsv;
                cv::Mat blueCones;
                cv::Mat blueConesOpen;
                cv::Mat blueConesClose;
                cv:: Mat yellowCones;
                cv:: Mat yellowConesOpen;
                cv:: Mat yellowConesClose;
                cv:: Mat topHalf;
              
                cv::Mat Kernel = cv::Mat(cv::Size(5,5),CV_8UC1,cv::Scalar(255));
                cvtColor(img,hsv,COLOR_BGR2HSV);
               
                inRange(hsv, Scalar(42,99,44),Scalar(155,200,79), blueCones);
               // inRange(hsv, Scalar(105,141,31),Scalar(130,235,255), blueCones);
                       
               
           //TODO: find range for yellow cones
                inRange(hsv, Scalar(18,101,104),Scalar(53,255,255), yellowCones);
                //inRange(hsv, Scalar(18,141,14),Scalar(34,235,255), yellowCones);

                inRange(hsv, Scalar(179,255,255),Scalar(179,255,255), topHalf);

                //result = blueCones;
                //result2 = blueCones + yellowCones;
                
                //Opening and closing are used for getting rid of noise
                // Much of the code between line 140 and 164 is not being used at the moment

                cv::morphologyEx(blueCones, blueConesOpen,cv::MORPH_OPEN,Kernel);
                cv::morphologyEx(blueConesOpen, blueConesClose,cv::MORPH_CLOSE,Kernel); 

                cv::morphologyEx(yellowCones, yellowConesOpen,cv::MORPH_OPEN,Kernel);
                cv::morphologyEx(yellowConesOpen, yellowConesClose,cv::MORPH_CLOSE,Kernel);

                cv::morphologyEx(blueCones, blueConesOpen,cv::MORPH_OPEN,Kernel);
                cv::morphologyEx(blueConesOpen, blueConesClose,cv::MORPH_CLOSE,Kernel); 

                cv::morphologyEx(yellowCones, yellowConesOpen,cv::MORPH_OPEN,Kernel);
                cv::morphologyEx(yellowConesOpen, yellowConesClose,cv::MORPH_CLOSE,Kernel);

                Mat r= blueConesClose + yellowConesClose;

                Mat back;

               //this is the short way
               cv::Rect myROI(0,200,640,280);
               //cv::Rect myROI(0,250,640,100);
               cv::Rect myROITop(0,100,640,200);
               //cv::rectangle(r, cv::Point(50, 50), cv::Point(200, 200), cv::Scalar(255,0,0)); 
               Mat croppedImg= r(myROI);   
               
                cv::Mat topHalfFinal(topHalf);
               cv::Mat croppedImageTop = topHalfFinal(myROITop);

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
                Mat gBlurredImgBlue;
                Mat dilatedImgBlue;
                Mat cannyDilateBlue;
                Mat warpedImgBlue;

                Mat gBlurredImgYellow;
                Mat dilatedImgYellow;
                Mat cannyDilateYellow;
                Mat warpedImgYellow;

                Mat warpedImgCombined;
                //Both the blue and the yellow cones are givven a gaussian blur, dilated, and put through the canny method
                //Canny detects the edges of a given image
                GaussianBlur(blueCones,gBlurredImgBlue,Size(5,5),0);
                dilate(gBlurredImgBlue, dilatedImgBlue, Mat(), Point(-1, -1), 2, 1, 1); 
                Canny(dilatedImgBlue, cannyDilateBlue, 127,255,3);

                GaussianBlur(yellowCones,gBlurredImgYellow,Size(5,5),0);
                dilate(gBlurredImgYellow, dilatedImgYellow, Mat(), Point(-1, -1), 2, 1, 1); 
                Canny(dilatedImgYellow, cannyDilateYellow, 127,255,3);

                vector <Point2f> src_1[6000];
                //These are the points from the original image that will be stretched out in the perspective warp.  
                //pts1 is the array of points.  
                vector <Point2f> pts1;
                pts1.push_back(Point2f(slider_x_left, slider_y));  //The x and y coordinates of the top two points can be adjusted with the
                pts1.push_back (Point2f(slider_x_right, slider_y)); //track bar
                pts1.push_back(Point2f(0, 386));
                pts1.push_back (Point2f(632, 386));
                //These are the corners of the new image that the section of the image within the previous points will be sretched into
                //These points basically determine the width and height of the new image which will be the same as the original image
                vector <Point2f> pts2;
                pts2.push_back(Point2f(0,0));
                pts2.push_back(Point2f(WIDTH,0));
                pts2.push_back(Point2f(0,HEIGHT));
                pts2.push_back(Point2f(WIDTH,HEIGHT));
                //The matrix in the points on the original image that are to be stretched out and the new points thew will be 
                //stretched out to
                Mat matrix = getPerspectiveTransform(pts1,pts2);
                
                //The warp perspective is what stretches out the selected region of the image
                warpPerspective(cannyDilateBlue, warpedImgBlue, matrix, img.size());
                warpPerspective(cannyDilateYellow, warpedImgYellow, matrix, img.size());
                //This combines the warped images for the blue and yellow cones.
                warpedImgCombined = warpedImgBlue + warpedImgYellow; 

                Mat cannyMany;
                Canny(warpedImgCombined, cannyMany,127, 255, 3);

                Mat toShow = warpedImgCombined;
                RNG rng(12345);
                
                Scalar color= Scalar(rng.uniform(0,225), rng.uniform(0,255), rng.uniform(0,255));
               
                vector<vector<Point> > contoursF;
                vector<vector<Point> > contoursY;

                findContours(warpedImgBlue, contoursF,RETR_TREE,CHAIN_APPROX_SIMPLE); 
                findContours(warpedImgYellow, contoursY,RETR_TREE,CHAIN_APPROX_SIMPLE);

                vector<vector<Point> > contour_poly1(contoursF.size() );
                vector<vector<Point> > contour_polyY(contoursY.size() );
                vector<Rect> boundRect1( contoursF.size() );
                vector<Rect> boundRectY( contoursY.size() );
                vector<Moments> mu2 (contoursF.size());
                vector<Moments> muY (contoursY.size());

                 for(size_t i=0; i<contoursF.size();i++){
                    mu2[i]= moments(contoursF[i], false);
                    approxPolyDP(contoursF[i], contour_poly1[i], 3, true); 
                    boundRect1[i]= boundingRect(contour_poly1[i]);
                 }
                  for(size_t i=0; i<contoursY.size();i++){
                    muY[i]= moments(contoursY[i], false);
                    approxPolyDP(contoursY[i], contour_polyY[i], 3, true); 
                    boundRectY[i]= boundingRect(contour_polyY[i]);
                 }

                Mat bro= Mat::zeros(cannyMany.size(), CV_8UC3);
                vector<Point2f> mcF (contoursF.size());
                vector<Point2f> mcY (contoursY.size());

                for(int unsigned i=0; i< contoursF.size();i++){
                    mcF[i]= Point2f(mu2[i].m10/mu2[i].m00, mu2[i].m01/mu2[i].m00);
                }

                for(int unsigned i=0; i< contoursY.size();i++){
                    mcY[i]= Point2f(muY[i].m10/muY[i].m00, muY[i].m01/muY[i].m00);
                }
                
                Point lineStart = Point(320, 400);
                for(int unsigned i =0; i<contoursF.size(); i++){
                    drawContours(bro, contour_poly1, (int)i, color);
                    rectangle(bro,boundRect1[i].tl(), boundRect1[i].br(), color,2);
                    circle(bro,mcF[i],4,color,-1,8,0);
                   /* if(i>0) {
                        line(bro, mcF[i-1], mcF[i], color,5 );
                    } */  
                    line(bro, lineStart, mcF[i], color, 5);
                
                }
                for(int unsigned i =0; i<contoursY.size(); i++){
                    drawContours(bro, contour_polyY, (int)i, color);
                    rectangle(bro,boundRectY[i].tl(), boundRectY[i].br(), color,2);
                    circle(bro,mcY[i],4,color,-1,8,0);
                    /*if(i>0) {
                        line(bro, mcY[i-1], mcY[i], color,5 );
                    }*/
                    line(bro, lineStart, mcY[i], Scalar(0,255,0), 5);    
                
                }
               /* vector<Vec2f> l;
                //an attempt to apply houghlines
                HoughLines(cannyMany,l, 1,CV_PI/180,150,0,0 );
               for(int unsigned i =0; l.size();i++){
                           float rho = l[i][0], theta = l[i][1];
                            Point pt1, pt2;
                            double a = cos(theta), w = sin(theta);
                            double x0 = a*rho, y0 = w*rho;
                            pt1.x = cvRound(x0 + 1000*(-w));
                            pt1.y = cvRound(y0 + 1000*(a));
                            pt2.x = cvRound(x0 - 1000*(-w));
                            pt2.y = cvRound(y0 - 1000*(a));
                            line( toShow, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);

               }*/


                //normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
                // If you want to access the latest received ground steering, don't forget to lock the mutex:

                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }
                
                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::imshow("warped image", toShow);
                    cv::imshow("with rect", bro);
                    //cv::imshow("with g blurr", gBlurredImg);
                    //cv::imshow("with dilation", dilatedImg);
                    cv::imshow("with dilation and canny", cannyDilateYellow);

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

