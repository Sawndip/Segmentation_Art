// sys
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  
#include <sys/types.h>  
#include <fcntl.h>  
#include <dirent.h> 
#include <string.h>

#include <iostream>
#include <fstream>
#include <sstream>
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
using std :: cin;
using std :: cout;
using std :: endl;
using namespace cv;
using namespace Seg_Three;

///////////////////// Code ///////////////////////////////////////////////////////////////
namespace
{
   
string intToString(const int n)
{
    char nameStr[20] = {0};
    snprintf(nameStr, sizeof(nameStr), "%d", n);
    std::string formatedStr(nameStr);
    return formatedStr; 
}

void collectImageSequenceFiles(string & imgFileFolder, vector <string> & imgNames,
                               const int startFrame)
{
    imgNames.clear();
    DIR *p_dir;   
    struct dirent *p_dirent;  
    if ((p_dir = opendir(imgFileFolder.c_str())) == NULL)
    {  
        fprintf(stderr, "--> can't open %s\n",imgFileFolder.c_str());  
        exit(0);  
    }
    int fileNum = 0;
    while ((p_dirent = readdir(p_dir)))
    {  
        fileNum++;
    }  
    fileNum -= 4; // ".", ".." is not included.
    LogD("Total Jpgs: %d.\n", fileNum);
    for (int k = startFrame; k < fileNum; k++)
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
    int startFrame = 0;
    if ((argv[2] != NULL))
        startFrame = atoi(argv[2]);
    vector<string> imgFilePathes;    
    collectImageSequenceFiles(imgFileFolder, imgFilePathes, startFrame);

    bool bInitPosition = false;
    SegControl seg;
    //   int init(const int width, const int height,
    //            const int skipTB, const int skipLR,
    //            const int scanBorderSizeTB, const int scanBorderSizeLR);
    cv::Size dsize (640, 480);
    seg.init(640, 480, 32, 32, 2, 2);
    vector<SegResults> segResults;
    for(int i = 0; i < (int)imgFilePathes.size(); i ++)
    {   // 0. prepare
        segResults.clear();
        Mat inFrame = imread(imgFilePathes[i]);
        //const static double fx = dsize.width / inFrame.cols;
        //const static double fy = dsize.height / inFrame.rows;
        Mat inFrameResize;
        cv::resize(inFrame, inFrameResize, dsize);
        Mat inFrameGray;
        cvtColor(inFrameResize, inFrameGray, CV_RGB2GRAY);
        
        // 1. start process:
        // NOTE: the release app won't care 'binaryFrame', it is only for debugging.
        if (seg.processFrame(inFrameGray, segResults) > 0)
        {
            //// Draw the detected objects
            cv::Mat & binaryFrame = seg.getBinaryFrame();
            for (int k = 0; k < (int)segResults.size(); k++)
            {
                //const int x = round(segResults[k].m_curBox.x / fx);
                //const int y = round(segResults[k].m_curBox.y / fy);
                //const int width = round(segResults[k].m_curBox.width / fx);
                //const int height = round(segResults[k].m_curBox.height / fx);
                //cv::Rect rect(x, y, width, height);
                putText(inFrameResize, intToString(segResults[k].m_objIdx),
                        cvPoint(segResults[k].m_curBox.x, segResults[k].m_curBox.y),
                        2, 2, CV_RGB(25,200,25));
                rectangle(inFrameResize, segResults[k].m_curBox, Scalar(200,0,0), 1);
                putText(binaryFrame, intToString(segResults[k].m_objIdx),
                        cvPoint(segResults[k].m_curBox.x, segResults[k].m_curBox.y),
                        2, 2, CV_RGB(25,200,25));
                rectangle(binaryFrame, segResults[k].m_curBox, Scalar(200,0,0), 1);
            }

            //putText(inFrame, intToString(i), cvPoint(0,20), 2, 1, CV_RGB(25,200,25));
            //putText(binaryFrame, intToString(i), cvPoint(0,20), 2, 1, CV_RGB(25,200,25));
            imshow("In", inFrameResize);
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
        inFrameResize.release();        
        inFrameGray.release();
        //getchar();
    } 

    return 0;
}
