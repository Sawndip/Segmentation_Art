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
#include "segUtil.h"
#include "segControl.h"

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
#define SEQ_FILE_MAX_NUM (490)
    
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
    for (int k = 75; k < SEQ_FILE_MAX_NUM; k++)
    {
        string strNum = intToString(k);
        imgNames.push_back(imgFileFolder + "/img" + strNum + ".jpg");
        //if (k < 10)
        //    imgNames.push_back(imgFileFolder + "/img00" + strNum + ".jpg");
        //else if (k < 100)
        //    imgNames.push_back(imgFileFolder + "/img0" + strNum + ".jpg");
        //else
        //    imgNames.push_back(imgFileFolder + "/img" + strNum + ".jpg");
    }

    return;
}

} // namespace

///////////////////// Test ///////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
    string imgFileFolder("./data");
    if (argv[1] != NULL)
        imgFileFolder = argv[1];
    
    vector<string> imgFilePathes;    
    collectImageSequenceFiles(imgFileFolder, imgFilePathes);

    bool bInitPosition = false;
    SegControl seg;
    //   int init(const int width, const int height,
    //            const int skipTB, const int skipLR,
    //            const int scanBorderSizeTB, const int scanBorderSizeLR);
    Mat frame = imread(imgFilePathes[0]);
    seg.init(frame.cols, frame.rows, 32, 32, 2, 2);
    vector<SegResults> segResults;
    for(int i = 0; i < (int)imgFilePathes.size(); i ++)
    {   // 0. prepare
        segResults.clear();
        Mat inFrame = imread(imgFilePathes[i]);
        Mat inFrameGray;
        cvtColor(inFrame, inFrameGray, CV_RGB2GRAY);        
        //printf ("read in frame: %d, path %s, frameColorSpaceType %d.\n", 
        //        i, imgFilePathes[i].c_str(), inFrame.type());
        
        // 1. start process:
        // NOTE: the release app won't care 'binaryFrame', it is only for debugging.
        if (seg.processFrame(inFrameGray, segResults) > 0)
        {
            //// Draw the detected objects
            cv::Mat & binaryFrame = seg.getBinaryFrame();
            for (int k = 0; k < (int)segResults.size(); k++)
            {
                putText(inFrame, intToString(segResults[k].m_objIdx),
                        cvPoint(segResults[k].m_curBox.x, segResults[k].m_curBox.y),
                        2, 2, CV_RGB(25,200,25));
                rectangle(inFrame, segResults[k].m_curBox, Scalar(200,0,0), 1);
                putText(binaryFrame, intToString(segResults[k].m_objIdx),
                        cvPoint(segResults[k].m_curBox.x, segResults[k].m_curBox.y),
                        1, 1, CV_RGB(25,200,25));
                rectangle(binaryFrame, segResults[k].m_curBox, Scalar(200,0,0), 1);
            }

            //putText(inFrame, intToString(i), cvPoint(0,20), 2, 1, CV_RGB(25,200,25));
            //putText(binaryFrame, intToString(i), cvPoint(0,20), 2, 1, CV_RGB(25,200,25));
            imshow("In", inFrame);
            //imshow("InGray", inFrameGray);            
            imshow("Bg", binaryFrame);
            if (bInitPosition == false)
            {
                cv::moveWindow("In", 15, 10);
                //cv::moveWindow("InGray", 660, 10);            
                //cv::moveWindow("Bg", 10, 660);
                cv::moveWindow("Bg", 670, 10);
                bInitPosition = true;
            }
        }

        //waitKey(0);
        waitKey(1);
        inFrame.release();
        inFrameGray.release();
        //getchar();
    } 

    return 0;
}
