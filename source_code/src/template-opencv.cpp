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
#include <opencv2/imgproc.hpp>
 
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
                cv::Mat img;
 
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

                cv:: Mat result;
                cv:: Mat result2;
                cv:: Mat result3;

                cv::Mat Kernel = cv::Mat(cv::Size(5,5),CV_8UC1,cv::Scalar(255));
                cvtColor(img,hsv,COLOR_BGR2HSV);
               
               //range for blue cones
                inRange(hsv, Scalar(42,99,44),Scalar(155,200,79), blueCones);
               // inRange(hsv, Scalar(105,141,31),Scalar(130,235,255), blueCones);
                       
                 //range for yellow cones
                inRange(hsv, Scalar(18,101,104),Scalar(53,255,255), yellowCones);

                inRange(hsv, Scalar(179,255,255),Scalar(179,255,255), topHalf);

                //result = blueCones;
                //result2 = blueCones + yellowCones;


                //morph open removes white noise
                //morph close removes black noise 
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
               cv::Rect myROI(0,200,640,250);
               cv::Rect myROITop(0,100,640,200);
               cv::rectangle(r, cv::Point(50, 50), cv::Point(200, 200), cv::Scalar(255,0,0)); 
               Mat croppedImg= r(myROI);   
               

               cv::Mat topHalfFinal(topHalf);
               cv::Mat croppedImageTop = topHalfFinal(myROITop);


            
                //the code here for the purpose of creating 
               Mat b=blueConesClose(myROI);
               Mat bWhole;
               cv::vconcat(croppedImageTop, b, bWhole);

               Mat y=yellowConesClose(myROI);
               Mat yWhole;
               cv::vconcat(croppedImageTop, y, yWhole);

                Mat cannyOutputB;
                Mat cannyOutputY;

                //creating vectors for the blue and yellow cones to create lines later
                vector<vector<Point> > contoursB;
                vector<vector<Point> > contoursY;
                vector<Vec4i> hierarchy;
                RNG rng(12345);
            

                //create gray image of the original image
                Canny(yWhole, cannyOutputY,127, 255, 3); 
                Canny(bWhole, cannyOutputB,127,255,3);


                 //outputs array of arrays, contours are basically the boundaries of a shape in (x,y), 
                 //CHAIN_APPROX_SIMPLE removes redundant coordinates
                findContours(cannyOutputB, contoursB,RETR_TREE,CHAIN_APPROX_SIMPLE);
                findContours(cannyOutputY, contoursY,RETR_TREE,CHAIN_APPROX_SIMPLE);


                //only needed for rectangle
                vector<Rect> boundRect( contoursB.size() ); 
                
              //  vector<vector<Point> > contour_poly(contours.size() ); //array of array
                vector<Moments> muB (contoursB.size());
                vector<Moments> muY (contoursY.size());


                for(size_t i=0; i<contoursB.size();i++){
                    muB[i]= moments(contoursB[i], false);
                   // approxPolyDP(contours[i], contour_poly[i], 3, true); //this method approximates a polygonal curve with specified precision
                    //boundRect[i]= boundingRect(contour_poly[i]); //boundingRect calculates the up-right bounding rectangle of a point set. 
                    // input of boundingRect is a curve of 2D pints 
                    //The functions approxPolyDP approximate a curve or a polygon with another curve/polygon 
                    //with less vertices so that the distance between them is less or equal to the specified precision. 
                }
                for(size_t i=0; i<contoursY.size();i++){
                    muY[i]= moments(contoursY[i],false);
                }          

                Mat drawingB= Mat::zeros(cannyOutputB.size(), CV_8UC3);
                Mat drawingY= Mat::zeros(cannyOutputY.size(), CV_8UC3);
                
                vector<Point2f> mcB (contoursB.size());
                for(int unsigned i=0; i< contoursB.size();i++){
                    mcB[i]= Point2f(muB[i].m10/muB[i].m00, muB[i].m01/muB[i].m00);
                }
                vector<Point2f> mcY (contoursY.size());
                for(int unsigned i=0; i< contoursY.size();i++){
                    mcY[i]= Point2f(muY[i].m10/muY[i].m00, muY[i].m01/muY[i].m00);
                }

                //vector<vector<Point> > l;
                //l= [0,0];
                Scalar color= Scalar(rng.uniform(0,225), rng.uniform(0,255), rng.uniform(0,255));
                for(int unsigned i =0; i<contoursB.size(); i++){
                   // Scalar color= Scalar(rng.uniform(0,225), rng.uniform(0,255), rng.uniform(0,255));
                   // drawContours(drawing, contour_poly, (int)i, color);
                    //rectangle(drawing,boundRect[i].tl(), boundRect[i].br(), color,2); // tl() is topleft corner, br() bottom right coner 
                    circle(drawingB,mcB[i],4,color,-1,8,0);    
                                
                    //polylines(drawing, mc[i],1, Scalar(0,255,0),2,8,0);
                    if(i>0){
                        line(img, mcB[i-1], mcB[i], color,5 );
                    }
                    
                }
                for(int unsigned i=0; i<contoursY.size(); i++){
                    circle(drawingY,mcY[i],4,color,-1,8,0); 
                    //line(drawingY, mcY[i], mcY[i+1], color,5 );
                     if(i>0){

                        line(img, mcY[i-1], mcY[i], color,5 );
                    }
                }
                Mat lol= drawingY+drawingB;

                //load video and automatically determine source coords based on cones
                //return ordered coordinates
                //obtain a consistent order of points
                //unpack the points
                //compute the width of the new image 
                //max difference between bottom and top x coords from left to right 
                //compute hight 
                //top right bottom right y coords 
                //compute perspective transform matrix and apply
                //getPerspectiveTransform()
                //warpPerspective()
                //return warped video feed 
                

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }
 
                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                  //  cv::imshow("the area of interest", croppedImg);
                    //cv::imshow("blue and yellow cones with rectangle", r);
                    //cv::imshow("with lines and circles", lol);

                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}