/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    string detectors[4] ={"AKAZE"};
    string descriptors[5] ={ "SIFT","BRIEF", "FREAK", "ORB", "AKAZE"};
    

    for (string detectorType: detectors)
    {
        for (string descriptorType: descriptors)
        {
            double t = (double)cv::getTickCount();
            cout << "detector: " << detectorType << ", descriptor: " << descriptorType << "\n";


            float n_keypoints= 0;
            float n_matches = 0;
            // data location
            string dataPath = "../";

            // camera
            string imgBasePath = dataPath + "images/";
            string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
            string imgFileType = ".png";
            int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
            int imgEndIndex = 9;   // last file index to load
            int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

            // misc
            int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
            vector<DataFrame> dataBuffer(dataBufferSize); // list of data frames which are held in memory at the same time
            bool bVis = false;            // visualize results

            /* MAIN LOOP OVER ALL IMAGES */

            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;

                for(int i=1; i<dataBufferSize; i++)
                {

                    dataBuffer[i-1] = dataBuffer[i];
                }
                dataBuffer[dataBufferSize-1] = frame;
                

                //dataBuffer.push_back(frame);

                //// EOF STUDENT ASSIGNMENT
                //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

                if (detectorType.compare("SHITOMASI") == 0)
                {
                    detKeypointsShiTomasi(keypoints, imgGray, false);
                }
                else if (detectorType.compare("HARRIS") == 0)
                {
                    detKeypointsHarris(keypoints, imgGray, false);
                }
                else
                {
                    detKeypointsModern(keypoints, imgGray, detectorType, false);
                }
                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;

                if (bFocusOnVehicle)
                {
                    cv::Mat mask = cv::Mat::zeros(imgGray.rows, imgGray.cols, CV_8U); // all 0
                    mask(cv::Rect(535, 180, 180, 150)) = 1;

                    cv::KeyPointsFilter::runByPixelsMask(keypoints, mask);
                    
                }
                //cout << " NUM keypoints after filtering " << keypoints.size() << endl;
                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;
                //cout << "#2 : DETECT KEYPOINTS done" << endl;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                if (imgIndex >= 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    string outputType = "DES_HOG"; // DES_BINARY, DES_HOG
                    string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

                    //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                    if((descriptorType.compare("SIFT")==0) || (descriptorType.compare("HARRIS")==0))
                    {
                        outputType = "DES_HOG";
                    }
                    else
                    {
                        outputType = "DES_BINARY";
                    }
                    cout<< "1 ";
                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                    matches, outputType, matcherType, selectorType);

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done " << matches.size() << endl;

                    // visualize matches between curre9nt and previous image
                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        //cout << "Press key to continue to next image" << endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;

                    n_keypoints += keypoints.size();
                    n_matches += matches.size();
                }

            } // eof loop over all images

            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            cout << " Keypoints n=" << n_keypoints/9 << " Matchs n=" << n_matches/9 << " time: " << 1000 * t / 9.0 << " ms" << endl<< endl<< endl;
        }
        
    }

    return 0;
}
