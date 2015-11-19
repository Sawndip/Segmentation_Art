// sys
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
// tools
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// project
#include "psoBook.h"

// namespaces
using std :: string;
using std :: cout;
using std :: endl;
using namespace cv;
using namespace Seg_Three;

///////////////////// Code ///////////////////////////////////////////////////////////////
namespace
{

#define SEQ_FILE_DIR ("./data")
#define SEQ_FILE_MAX_NUM (500)
    
string intToString(const int n)
{
    char nameStr[20] = {0};
    snprintf(nameStr, sizeof(nameStr), "%d", n);
    std::string formatedStr(nameStr);
    return formatedStr; 
}

void collectImageSequenceFiles(string & imgFileFolder, vector <string> & imgNames)
{
    imgNames.clear();
    for (int k = 0; k < SEQ_FILE_MAX_NUM; k++)
    {
        string strNum = intToString(k);
        if (k < 10)
            imgNames.push_back(imgFileFolder + "/img00" + strNum + ".jpg");
        else if (k < 100)
            imgNames.push_back(imgFileFolder + "/img0" + strNum + ".jpg");
        else
            imgNames.push_back(imgFileFolder + "/img" + strNum + ".jpg");
    }

    printf("Test Images: %s - %s.\n", imgNames[0].c_str(),
           imgNames[SEQ_FILE_MAX_NUM - 1].c_str());
    return;
}

} // namespace

///////////////////// Test ///////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
    string imgFileFolder("./data");
    vector<string> imgFilePathes;    
    collectImageSequenceFiles(imgFileFolder, imgFilePathes);

    PsoBook psoBook;
    psoBook.init(640, 480);
    for(int i = 0; i < (int)imgFilePathes.size(); i++)
    {
        //Mat readFrame = imread(imgFilePathes[i]);
        //Mat inFrame;         cvtColor(readFrame, inFrame, CV_BGR2Lab);
        Mat inFrame = imread(imgFilePathes[i]);
        printf ("read in frame: %d, path %s, frameColorSpaceType %d.\n", 
                i, imgFilePathes[i].c_str(), inFrame.type());
        Mat binaryFrame(480, 640, CV_8UC1);
        // 
        if (psoBook.processFrame(inFrame, binaryFrame) > 0)
        {
            //// Draw the detected objects
            //for (int k  0; k < (int)rects.size(); k++)
            //    rectangle(frame, rects[k], Scalar(200,0,0), 2);
            putText(inFrame, intToString(i), cvPoint(0,20), 2, 1, CV_RGB(25,200,25));
            //putText(binaryFrame, intToString(i), cvPoint(0,20), 2, 1, CV_RGB(25,200,25));
            imshow("PsoSegment", inFrame);
            imshow("bgfg", binaryFrame);
            cv::moveWindow("PsoSegment", 10, 10);
            cv::moveWindow("bgfg", 660, 10);             
        }

        //waitKey(0);
        waitKey(1);
        inFrame.release();
        binaryFrame.release();
        //getchar();
    } 

    return 0;
}
