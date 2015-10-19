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
// project
#include "artsegment.h"

// namespaces
using std :: string;
using std :: cout;
using std :: endl;
using namespace cv;
using namespace Art_Segment;

///////////////////// Code ///////////////////////////////////////////////////////////////
namespace
{

#define SEQ_FILE_DIR ("./data")
#define SEQ_FILE_MAX_NUM (484)
    
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
            imgNames.push_back(imgFilePath + "/img0000" + strNum + ".png");
        else if (k < 100)
            imgNames.push_back(imgFilePath + "/img000" + strNum + ".png");
        else
            imgNames.push_back(imgFilePath + "/img00" + strNum + ".png");        
    }

    printf("Through %s - %s.\n", imgNames[0].c_str(),
           imgNames[SEQ_FILE_MAX_NUM].c_str());
    return;
}

} // namespace

///////////////////// Test ///////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
    string imgFileFolder("./data");
    vector<string> imgFilePathes;    
    collectImageSequenceFiles(imgFileFolder, imgFilePathes);

    ArtSegment asn;
    int count = 0;
    for(int i = 0; i < (int)imgNames.size(); i ++)
    {
        Mat frame = imread(imgFilePathes[i]);

        vector<Rect> & rects = asn.processFrame(frame);
        // Draw the detected objects
        for (int k  0; k < (int)rects.size(); k++)
            rectangle(frame, rects[k], Scalar(200,0,0), 2); 

        putText(frame, intToString(i), cvPoint(0,20), 2, 1, CV_RGB(25,200,25));

        imshow("ArtSegment", frame); 
        waitKey(1);
    } 

    return 0;
}
